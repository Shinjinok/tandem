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
#include <AP_Arming/AP_Arming.h>


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
  float pressure_pascal;
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
    //WITH_SEMAPHORE(state.sem);

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
        for( uint64_t i = 0; i< wp - sizeof(gn_pkt_header) +1;i++){
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

    #if AP_BARO_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = pressure_pa;
        baro.temperature = 25.0f;
        //uart->printf("baro %f\n",pkt1.pressure_pascal);
        AP::baro().handle_external(baro);
    } 
    #endif

    #if AP_COMPASS_EXTERNALAHRS_ENABLED
        {
            AP_ExternalAHRS::mag_data_message_t mag;
            mag.field = pkt1_mag;
            AP::compass().handle_external(mag);
        } 
    #endif 

    {
        AP_ExternalAHRS::ins_data_message_t ins;
        ins.accel = accel;
        ins.gyro = gyro;
        ins.temperature = 25.0f;
        AP::ins().handle_external(ins);
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
        return "FlightGear";
    }
    return nullptr;
}

/*
  process packet type 1
 */
void AP_ExternalAHRS_FlightGear::process_packet1(const uint8_t *b)
{
    AP_Arming *ap_arming = AP_Arming::get_singleton();
    if(ap_arming->can.alive && ap_arming->can.my_id == 10){
        uint16_t values[8] {};
        hal.rcout->read(values, 8);
        uart->printf(":%d:%d:%d:%d:%d:%d:%d:%d:\n",values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7]);
    }
    if(!ap_arming->can.alive && ap_arming->can.my_id == 11){
        uint16_t values[8] {};
        hal.rcout->read(values, 8);
        uart->printf(":%d:%d:%d:%d:%d:%d:%d:%d:\n",values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7]);
    }
    //uart->write((const uint8_t *) &values[0],sizeof(values));
    //uart->printf("\n");


    const struct Generic_packet &pkt1 = *(struct Generic_packet *)b;
    
    last_pkt1_ms = AP_HAL::millis();


    int32_t lat = pkt1.lat_lon[0] * 1.0e7;
    int32_t lon = pkt1.lat_lon[1] * 1.0e7;
    int32_t alt = pkt1.alt_m * 1.0e2;
    Location loc = Location{lat,lon,alt,Location::AltFrame::ABSOLUTE};
    accel = Vector3f{pkt1.pilot_accel_swu_xyz_mps[0], pkt1.pilot_accel_swu_xyz_mps[1], pkt1.pilot_accel_swu_xyz_mps[2]};
    gyro = Vector3f{pkt1.pqr_rad[0], pkt1.pqr_rad[1], pkt1.pqr_rad[2]};
    pkt1_mag = Vector3f(pkt1.mag[0],pkt1.mag[1],pkt1.mag[2]);
    pressure_pa = pkt1.pressure_pascal;
    Vector3f velocity = Vector3f{pkt1.speed_ned_mps[0], pkt1.speed_ned_mps[1], pkt1.speed_ned_mps[2]};
    Quaternion quat;
    quat.from_euler(pkt1.rpy_rad[0], pkt1.rpy_rad[1], pkt1.rpy_rad[2]);
    
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro = gyro; 
        state.quat = quat;
        state.have_quaternion = true;
        state.velocity = velocity;
        state.have_velocity = true;

        state.location = loc;
        state.have_location = true;

        if (!state.have_origin) {
            state.origin = loc;
            state.have_origin = true;
        } 
        
    }
     AP_ExternalAHRS::gps_data_message_t gps;
    // get ToW in milliseconds
    gps.gps_week = (uint16_t) ((uint64_t) pkt1.timestamp / AP_MSEC_PER_WEEK);
    gps.ms_tow = (uint32_t) ((uint64_t) pkt1.timestamp % AP_MSEC_PER_WEEK);
    gps.fix_type = 5;
    gps.satellites_in_view = 100;

    gps.horizontal_pos_accuracy = 0.0005f;
    gps.vertical_pos_accuracy = 0.0005f;
    gps.horizontal_vel_accuracy = 0.0005f;

    gps.hdop = 1.0f;
    gps.vdop = 1.0f;
    gps.latitude = lat;
    gps.longitude = lon;
    gps.msl_altitude = alt;

    gps.ned_vel_north = velocity.x;
    gps.ned_vel_east = velocity.y;
    gps.ned_vel_down = velocity.z;

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gyro %f %f %f",gyro.x,gyro.y,gyro.z);






   

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eahra ms %f ",gyro.x,gyro.y,gyro.z);


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
 
/*      AP::logger().WriteStreaming("EAH1", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lon,Alt",
                       "sdddnnnDUm", "F000000GG0",
                       "QffffffLLf", AP_HAL::micros64(),
                       pkt1.rpy_rad[0], pkt1.rpy_rad[1], pkt1.rpy_rad[2],
                       velocity.x, velocity.y, velocity.z, lat, lon, alt); */
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
   
    return (now - last_pkt1_ms < 100);
}

bool AP_ExternalAHRS_FlightGear::initialised(void) const
{
    if (!setup_complete) {
        return false;
    }

    return last_pkt1_ms != 0 ;
}

bool AP_ExternalAHRS_FlightGear::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AP_ExternalAHRS_FlightGear setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "AP_ExternalAHRS_FlightGear unhealthy");
        return false;
    }

    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_FlightGear::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
   
    if (last_pkt1 !=0 ) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_pkt1 !=0 ) {
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
    const float vel_gate = 10; // represents hz value data is posted at
    const float pos_gate = 10; // represents hz value data is posted at
    const float hgt_gate = 10; // represents hz value data is posted at
    const float mag_var = 10; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       0.01/vel_gate, 
                                       0.01/pos_gate, 
                                       0.01/hgt_gate,
                                       mag_var, 0, 0);
} 

#endif  // AP_EXTERNAL_AHRS_FLIGHTGEAR_ENABLED
