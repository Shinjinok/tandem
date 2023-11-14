/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  suppport for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_FLIGHTGEAR_ENABLED

#include "AP_ExternalAHRS_FlightGear.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>


extern const AP_HAL::HAL &hal;




static const uint8_t gn_pkt_header[] { 0xFE, 0xBB, 0xAA};

struct PACKED Generic_packet {
  double timestamp;  // in seconds
  double lat_lon[2];
  float alt_m;
  float pqr_rad[3];
  float pilot_accel_swu_xyz_mps[3];
  float speed_ned_mps[3];
  float rpy_rad[3];
  float pressure_mbar;
  float rpm;
  float mag[3];
};


// constructor
AP_ExternalAHRS_FlightGear::AP_ExternalAHRS_FlightGear(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    //bufsize = 1024;
    pktbuf = new uint8_t[bufsize];

    last_pkt1 = new Generic_packet;
    // last_pkt2 = new VN_packet2;

    if (!pktbuf || !last_pkt1) {
        AP_BoardConfig::allocation_error("ExternalAHRS");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_FlightGear::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");

 
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
//#define SYNC_BYTE 0xFA
/* !!!!!!  little endian*/
bool AP_ExternalAHRS_FlightGear::check_uart() 
{
    if (!setup_complete) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);

    uint32_t n = uart->available();
 
    if (n < sizeof(gn_pkt_header)) {
        return false;
    }

    //uart->printf("n %ld\n",n);
    ssize_t nread = 0;
    if (wp + n< bufsize) {
          nread = uart->read(&pktbuf[wp], n);
          wp += nread;
    } 
    else {
        // overflow buffer reset
        wp = 0;
        match_header1 = false;
        return false;
    }

    


    if(!match_header1 && wp > 2){
        for( int16_t i = 0; i< wp - sizeof(gn_pkt_header) +1;i++){
            match_header1 = (0 == memcmp(&pktbuf[i], gn_pkt_header, sizeof(gn_pkt_header)));
            if(match_header1){
                int16_t remain = wp - i;
                if (i > 0) memmove(&pktbuf[0],&pktbuf[i],remain);
                wp = remain;
                
                //uart->printf("matching i= %d wp = %d\n",i,wp);
                break;
            }
            
        }
    }

    if(wp >= sizeof(Generic_packet) + sizeof(gn_pkt_header)){
        //uart->printf("wp %d\n",wp);
        process_packet1(&pktbuf[sizeof(gn_pkt_header)]);
        int remain = wp - sizeof(Generic_packet) - sizeof(gn_pkt_header);
        if(remain != 0) memmove(&pktbuf[0],&pktbuf[wp],remain);
        wp =  remain;
        match_header1 = false;
    }

    return true;
}

// Send command to read given register number and wait for responce
// Only run from thread! This blocks until a responce is received
#define READ_REQUEST_RETRY_MS 500
void AP_ExternalAHRS_FlightGear::wait_register_responce(const uint8_t register_num)
{
    nmea.register_number = register_num;

    uint32_t request_sent = 0;
    while (true) {
        hal.scheduler->delay(1);

        const uint32_t now = AP_HAL::millis();
        if (now - request_sent > READ_REQUEST_RETRY_MS) {
            // Send request to read
            nmea_printf(uart, "$%s%u", "VNRRG,", nmea.register_number);
            request_sent = now;
        }

        int16_t nbytes = uart->available();
        while (nbytes-- > 0) {
            char c = uart->read();
            if (decode(c)) {
                return;
            }
        }
    }
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_ExternalAHRS_FlightGear::decode(char c)
{
    switch (c) {
    case ',':
        // end of a term, add to checksum
        nmea.checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        if (nmea.sentence_done) {
            return false;
        }
        if (nmea.term_is_checksum) {
            nmea.sentence_done = true;
            uint8_t checksum = 16 * char_to_hex(nmea.term[0]) + char_to_hex(nmea.term[1]);
            return ((checksum == nmea.checksum) && nmea.sentence_valid);
        }

        // null terminate and decode latest term
        nmea.term[nmea.term_offset] = 0;
        if (nmea.sentence_valid) {
            nmea.sentence_valid = decode_latest_term();
        }

        // move onto next term
        nmea.term_number++;
        nmea.term_offset = 0;
        nmea.term_is_checksum = (c == '*');
        return false;
    }

    case '$': // sentence begin
        nmea.sentence_valid = true;
        nmea.term_number = 0;
        nmea.term_offset = 0;
        nmea.checksum = 0;
        nmea.term_is_checksum = false;
        nmea.sentence_done = false;
        return false;
    }

    // ordinary characters are added to term
    if (nmea.term_offset < sizeof(nmea.term) - 1) {
        nmea.term[nmea.term_offset++] = c;
    }
    if (!nmea.term_is_checksum) {
        nmea.checksum ^= c;
    }

    return false;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_ExternalAHRS_FlightGear::decode_latest_term()
{
    switch (nmea.term_number) {
        case 0:
            if (strcmp(nmea.term, "VNRRG") != 0) {
                return false;
            }
            break;

        case 1:
            if (nmea.register_number != strtoul(nmea.term, nullptr, 10)) {
                return false;
            }
            break;

        case 2:
            strncpy(model_name, nmea.term, sizeof(model_name));
            break;

        default:
            return false;
    }
    return true;
}

void AP_ExternalAHRS_FlightGear::update_thread()
{
    // Open port in the thread
    uart->begin(baudrate, 1024, 512);

    type = TYPE::FG;

    setup_complete = true;
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

const char* AP_ExternalAHRS_FlightGear::get_name() const
{
    if (setup_complete) {
        return model_name;
    }
    return nullptr;
}

/*
  process packet type 1
 */
void AP_ExternalAHRS_FlightGear::process_packet1(const uint8_t *b)
{
    const struct Generic_packet &pkt1 = *(struct Generic_packet *)b;
    
    last_pkt1_ms = AP_HAL::millis();
    
    AP_ExternalAHRS::gps_data_message_t gps;
    // get ToW in milliseconds
        gps.gps_week = (uint64_t) pkt1.timestamp / AP_MSEC_PER_WEEK ;
        gps.ms_tow = (uint64_t)pkt1.timestamp % (60*60*24*7*1000ULL);
        gps.fix_type = 4;
        gps.satellites_in_view = 16;

        gps.horizontal_pos_accuracy = 0.01f;
        gps.vertical_pos_accuracy = 0.01f;
        gps.horizontal_vel_accuracy = 0.01f;

        gps.hdop = 1.0f;
        gps.vdop = 1.0f;
        gps.latitude = pkt1.lat_lon[0] * 1.0e7;
        gps.longitude = pkt1.lat_lon[1] * 1.0e7;
        gps.msl_altitude = pkt1.alt_m * 1.0e2;

        gps.ned_vel_north = pkt1.speed_ned_mps[0];
        gps.ned_vel_east = pkt1.speed_ned_mps[1];
        gps.ned_vel_down = pkt1.speed_ned_mps[2];

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
    
    {
        WITH_SEMAPHORE(state.sem);
        
        state.accel = Vector3f{pkt1.pilot_accel_swu_xyz_mps[0], pkt1.pilot_accel_swu_xyz_mps[1], pkt1.pilot_accel_swu_xyz_mps[2]};
        state.gyro = Vector3f{pkt1.pqr_rad[0], pkt1.pqr_rad[1], pkt1.pqr_rad[2]};
        
        state.quat.from_euler(pkt1.rpy_rad[0], pkt1.rpy_rad[1], pkt1.rpy_rad[2]);
        state.have_quaternion = true;
        state.velocity = Vector3f{pkt1.speed_ned_mps[0], pkt1.speed_ned_mps[1], pkt1.speed_ned_mps[2]};
        state.have_velocity = true;

        state.location = Location{int32_t(pkt1.lat_lon[0] * 1.0e7),
                                  int32_t(pkt1.lat_lon[1] * 1.0e7),
                                  int32_t(pkt1.alt_m * 1.0e2),
                                  Location::AltFrame::ABSOLUTE};
        state.have_location = true; 

        if (gps.fix_type >= 3 && !state.have_origin) {
            state.origin = Location{int32_t(pkt1.lat_lon[0] * 1.0e7),
                                  int32_t(pkt1.lat_lon[1] * 1.0e7),
                                  int32_t(pkt1.alt_m * 1.0e2),
                                  Location::AltFrame::ABSOLUTE};
            state.have_origin = true;
        }
    }
        uint16_t values[8] {};
        hal.rcout->read(values, 8);
        uart->printf(":%d:%d:%d:%d:%d:%d:%d:%d:\n",values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7]);
        //uart->write((const uint8_t *) &values[0],sizeof(values));
        //uart->printf("\n");


#if AP_BARO_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = pkt1.pressure_mbar*1e3;
        baro.temperature = 25.0f;
        //uart->printf("baro %f\n",pkt1.pressure);
        AP::baro().handle_external(baro);
    } 
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
     {
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = Vector3f{pkt1.mag[0], pkt1.mag[1], pkt1.mag[2]};
        mag.field *= 1000; // to mGauss

        AP::compass().handle_external(mag);
    } 
#endif 

    {
        AP_ExternalAHRS::ins_data_message_t ins;

        ins.accel = state.accel;
        ins.gyro = state.gyro;
        ins.temperature = 25.0f;

        AP::ins().handle_external(ins);
    }


    // @LoggerMessage: EAH1
    // @Description: External AHRS data
    // @Field: TimeUS: Time since system startup
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Alt: altitude AMSL
    // @Field: UXY: uncertainty in XY position
    // @Field: UV: uncertainty in velocity
    // @Field: UR: uncertainty in roll
    // @Field: UP: uncertainty in pitch
    // @Field: UY: uncertainty in yaw

/*     AP::logger().WriteStreaming("EAH1", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lon,Alt,UXY,UV,UR,UP,UY",
                       "sdddnnnDUmmnddd", "F000000GG000000",
                       "QffffffLLffffff",
                       AP_HAL::micros64(),
                       pkt1.ypr[2], pkt1.ypr[1], pkt1.ypr[0],
                       pkt1.velNED[0], pkt1.velNED[1], pkt1.velNED[2],
                       int32_t(pkt1.positionLLA[0]*1.0e7), int32_t(pkt1.positionLLA[1]*1.0e7),
                       float(pkt1.positionLLA[2]),
                       pkt1.posU, pkt1.velU,
                       pkt1.yprU[2], pkt1.yprU[1], pkt1.yprU[0]); */
}

/*
  process packet type 2
 */
void AP_ExternalAHRS_FlightGear::process_packet2(const uint8_t *b)
{
    /* const struct VN_packet2 &pkt2 = *(struct VN_packet2 *)b;
    const struct Generic_packet &pkt1 = *last_pkt1;

    last_pkt2_ms = AP_HAL::millis();
    *last_pkt2 = pkt2;

    AP_ExternalAHRS::gps_data_message_t gps;

    // get ToW in milliseconds
    gps.gps_week = pkt2.timeGPS / (AP_MSEC_PER_WEEK * 1000000ULL);
    gps.ms_tow = (pkt2.timeGPS / 1000000ULL) % (60*60*24*7*1000ULL);
    gps.fix_type = pkt2.GPS1Fix;
    gps.satellites_in_view = pkt2.numGPS1Sats;

    gps.horizontal_pos_accuracy = pkt1.posU;
    gps.vertical_pos_accuracy = pkt1.posU;
    gps.horizontal_vel_accuracy = pkt1.velU;

    gps.hdop = pkt2.GPS1DOP[4];
    gps.vdop = pkt2.GPS1DOP[3];

    gps.latitude = pkt2.GPS1posLLA[0] * 1.0e7;
    gps.longitude = pkt2.GPS1posLLA[1] * 1.0e7;
    gps.msl_altitude = pkt2.GPS1posLLA[2] * 1.0e2;

    gps.ned_vel_north = pkt2.GPS1velNED[0];
    gps.ned_vel_east = pkt2.GPS1velNED[1];
    gps.ned_vel_down = pkt2.GPS1velNED[2];

    if (gps.fix_type >= 3 && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(pkt2.GPS1posLLA[0] * 1.0e7),
                                int32_t(pkt2.GPS1posLLA[1] * 1.0e7),
                                int32_t(pkt2.GPS1posLLA[2] * 1.0e2),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }
    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    } */
}

/*
  process VN-100 packet type 1
 */
void AP_ExternalAHRS_FlightGear::process_packet_VN_100(const uint8_t *b)
{
   /*  const struct VN_100_packet1 &pkt = *(struct VN_100_packet1 *)b;

    last_pkt1_ms = AP_HAL::millis();

    //const bool use_uncomp = option_is_set(AP_ExternalAHRS::OPTIONS::VN_UNCOMP_IMU);
    const bool use_uncomp = true;

    {
        WITH_SEMAPHORE(state.sem);
        if (use_uncomp) {
            state.accel = Vector3f{pkt.uncompAccel[0], pkt.uncompAccel[1], pkt.uncompAccel[2]};
            state.gyro = Vector3f{pkt.uncompAngRate[0], pkt.uncompAngRate[1], pkt.uncompAngRate[2]};
        } else {
            state.accel = Vector3f{pkt.accel[0], pkt.accel[1], pkt.accel[2]};
            state.gyro = Vector3f{pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]};
        }

        state.quat = Quaternion{pkt.quaternion[3], pkt.quaternion[0], pkt.quaternion[1], pkt.quaternion[2]};
        state.have_quaternion = true;
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = pkt.pressure*1e3;
        baro.temperature = pkt.temp;

        AP::baro().handle_external(baro);
    }
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::mag_data_message_t mag;
        if (use_uncomp) {
            mag.field = Vector3f{pkt.uncompMag[0], pkt.uncompMag[1], pkt.uncompMag[2]};
        } else {
            mag.field = Vector3f{pkt.mag[0], pkt.mag[1], pkt.mag[2]};
        }
        mag.field *= 1000; // to mGauss

        AP::compass().handle_external(mag);
    }
#endif

    {
        AP_ExternalAHRS::ins_data_message_t ins;

        ins.accel = state.accel;
        ins.gyro = state.gyro;
        ins.temperature = pkt.temp;

        AP::ins().handle_external(ins);
    }

    // @LoggerMessage: EAH3
    // @Description: External AHRS data
    // @Field: TimeUS: Time since system startup
    // @Field: Temp: Temprature
    // @Field: Pres: Pressure
    // @Field: MX: Magnetic feild X-axis
    // @Field: MY: Magnetic feild Y-axis
    // @Field: MZ: Magnetic feild Z-axis
    // @Field: AX: Acceleration X-axis
    // @Field: AY: Acceleration Y-axis
    // @Field: AZ: Acceleration Z-axis
    // @Field: GX: Rotation rate X-axis
    // @Field: GY: Rotation rate Y-axis
    // @Field: GZ: Rotation rate Z-axis
    // @Field: Q1: Attitude quaternion 1
    // @Field: Q2: Attitude quaternion 2
    // @Field: Q3: Attitude quaternion 3
    // @Field: Q4: Attitude quaternion 4

    AP::logger().WriteStreaming("EAH3", "TimeUS,Temp,Pres,MX,MY,MZ,AX,AY,AZ,GX,GY,GZ,Q1,Q2,Q3,Q4",
                       "sdPGGGoooEEE----", "F000000000000000",
                       "Qfffffffffffffff",
                       AP_HAL::micros64(),
                       pkt.temp, pkt.pressure*1e3,
                       use_uncomp ? pkt.uncompMag[0] : pkt.mag[0],
                       use_uncomp ? pkt.uncompMag[1] : pkt.mag[1], 
                       use_uncomp ? pkt.uncompMag[2] : pkt.mag[2],
                       state.accel[0], state.accel[1], state.accel[2],
                       state.gyro[0], state.gyro[1], state.gyro[2],
                       state.quat[0], state.quat[1], state.quat[2], state.quat[3]);
 */
}


// get serial port number for the uart
int8_t AP_ExternalAHRS_FlightGear::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_FlightGear::healthy(void) const
{
    const uint32_t now = AP_HAL::millis();
   /*  if (type == TYPE::VN_100) {
        return (now - last_pkt1_ms < 40);
    } */
    if (type == TYPE::FG) {
        return (now - last_pkt1_ms < 200);
    }
    return (now - last_pkt1_ms < 200);
}

bool AP_ExternalAHRS_FlightGear::initialised(void) const
{
    if (!setup_complete) {
        return false;
    }
/*     if (type == TYPE::VN_100) {
        return last_pkt1_ms != 0;
    } */
    if (type == TYPE::FG) {
        return last_pkt1_ms != 0;
    }
    return last_pkt1_ms != 0 ;
}

bool AP_ExternalAHRS_FlightGear::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FlightGear setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FlightGear unhealthy");
        return false;
    }
/*     if (type == TYPE::VN_300) {
        if (last_pkt2->GPS1Fix < 3) {
            hal.util->snprintf(failure_msg, failure_msg_len, "FlightGear no GPS1 lock");
            return false;
        }
        if (last_pkt2->GPS2Fix < 3) {
            hal.util->snprintf(failure_msg, failure_msg_len, "FlightGear no GPS2 lock");
            return false;
        }
    } */
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_FlightGear::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (type == TYPE::FG) {
        if (last_pkt1) {
            status.flags.initalized = 1;
        }
        if (healthy()) {
            status.flags.attitude = 1;
            status.flags.vert_vel = 1;
            status.flags.vert_pos = 1;
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
            
        }
    } else {
        status.flags.initalized = initialised();
        if (healthy()) {
            status.flags.attitude = true;
        }
    }
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_FlightGear::send_status_report(GCS_MAVLINK &link) const
{
    if (!last_pkt1) {
        return;
    }
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
 /*   const struct VN_packet1 &pkt1 = *(struct VN_packet1 *)last_pkt1;
    const float vel_gate = 5;
    const float pos_gate = 5;
    const float hgt_gate = 5;
    const float mag_var = 0;
     mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       pkt1.velU/vel_gate, pkt1.posU/pos_gate, pkt1.posU/hgt_gate,
                                       mag_var, 0, 0);
                                       */
} 

#endif  // AP_EXTERNAL_AHRS_FLIGHTGEAR_ENABLED
