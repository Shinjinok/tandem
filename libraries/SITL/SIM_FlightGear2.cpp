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
  simulator connector for ardupilot version of FlightGear2
*/

#include "SIM_FlightGear2.h"

#if HAL_SIM_FLIHGTGEAR2_ENABLED

#include <stdio.h>
#include <errno.h>

namespace SITL {

FlightGear2::FlightGear2(const char *frame_str) :
    Aircraft(frame_str),
    last_timestamp(0),
    socket_sitl{true}
{
    fprintf(stdout, "Starting SITL FlightGear2\n");
}

/*
  Create and set in/out socket
*/
void FlightGear2::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // FlightGear2 keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!socket_sitl.bind("0.0.0.0", port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", port_in);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    _FlightGear2_address = address;
    _FlightGear2_port = port_out;
    printf("Setting FlightGear2 interface to %s:%d \n", _FlightGear2_address, _FlightGear2_port);
}

/*
  decode and send servos
*/
void FlightGear2::send_servos(const struct sitl_input &input)
{
   udp_in_packet pkt;

    pkt.serveo[0] = (input.servos[0]-1500) / 500.0f ;
    pkt.serveo[1] = -(input.servos[1]-1500) / 500.0f ;
    pkt.serveo[2] = (2000-input.servos[2]) / 1000.0f;
    pkt.serveo[3] = (input.servos[3]-1500) / 500.0f ;
    pkt.serveo[4] = (2000-input.servos[2]) / 1000.0f;
    

    /*pkt.serveo[0] = (ch[0] -0.5)*2.0;
    pkt.serveo[1] = (-ch[1] +0.5)*2.0;
    pkt.serveo[2] = 1.0- ch[2];
    pkt.serveo[3] = (ch[3]-0.5)*2.0;
    pkt.serveo[4] = 1.0- ch[2];*/


    uint32_t data[5];
    data[0] = __bswap_32(pkt.data[0]);
    data[1] = __bswap_32(pkt.data[1]);
    data[2] = __bswap_32(pkt.data[2]);
    data[3] = __bswap_32(pkt.data[3]);
    data[4] = __bswap_32(pkt.data[4]);
    socket_sitl.sendto(&data, sizeof(data), _FlightGear2_address, _FlightGear2_port);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void FlightGear2::recv_fdm(const struct sitl_input &input)
{
    U_packet pkt;
    D_packet dp;
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    uint32_t now = AP_HAL::millis();
   // uint32_t taken_ms = now - last_one_hz_ms;
    last_one_hz_ms = now;
    while (socket_sitl.recv(&dp, RCV_SIZE, 1) != RCV_SIZE) {
        //send_servos(input);
        // Reset the timestamp after a long disconnection, also catch FlightGear2 reset
        if (get_wall_time_us() > last_wall_time_us + FlightGear2_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }
    

    for (long unsigned int i=0; i < NUM_64_DATA; i++){
        pkt.dp.data64[i] = __bswap_64(dp.data64[i]);
    }

    for (long unsigned int i=0; i < NUM_32_DATA; i++){
        pkt.dp.data32[i] = __bswap_32(dp.data32[i]);
    }

    const double deltat = pkt.g_packet.timestamp - last_timestamp;  // in seconds

    if (deltat < 0) {  // don't use old packet
        time_now_us += 1;
        return;
    }
    
    accel_body = Vector3f(pkt.g_packet.pilot_accel_swu_xyz[0]* FEET_TO_METERS,
                          pkt.g_packet.pilot_accel_swu_xyz[1]* FEET_TO_METERS,
                          pkt.g_packet.pilot_accel_swu_xyz[2]* FEET_TO_METERS);

    gyro = Vector3f(pkt.g_packet.pqr_rad[0],
                    pkt.g_packet.pqr_rad[1],
                    pkt.g_packet.pqr_rad[2]);

    velocity_ef = Vector3f(pkt.g_packet.speed_ned_fps[0]*FEET_TO_METERS,
                           pkt.g_packet.speed_ned_fps[1]*FEET_TO_METERS,
                           pkt.g_packet.speed_ned_fps[2]*FEET_TO_METERS);
    
 //   float roll,pitch,yaw;
 //   dcm.to_euler(&roll,&pitch,&yaw);

    location.lat = pkt.g_packet.lat_lon[0] * 1.0e7;
    location.lng = pkt.g_packet.lat_lon[1] * 1.0e7;
    location.alt = pkt.g_packet.alt * 100.0f + 500.0f;

    position = origin.get_distance_NED_double(location);
   // Vector3d home_pos = origin.get_distance_NED_double(home);

     // compute dcm from imu orientation
    dcm.from_euler(pkt.g_packet.orientation_rpy_deg[0]*DEG_TO_RAD_DOUBLE,
                   pkt.g_packet.orientation_rpy_deg[1]*DEG_TO_RAD_DOUBLE,
                   pkt.g_packet.orientation_rpy_deg[2]*DEG_TO_RAD_DOUBLE);
    //Matrix3f dcm2 = dcm;
    
   // Vector3f velocity_t = dcm2.transposed() * velocity_ef;
    //rpm[0] = pkt.g_packet.rpm;
    //rpm[1] = pkt.g_packet.rpm;
    //smooth_sensors();                    
    
    
   //printf("deltat %f  ms: %d----------------------------------\n",deltat,taken_ms);
  
  // printf("A: %3.3f %3.3f %3.3f G: %3.3f %3.3f %3.3f V:%f %f %f\n",accel_body.x,accel_body.y,accel_body.z,
   //                                             gyro.x,gyro.y,gyro.z, velocity_ef.x, velocity_ef.y,velocity_ef.z);
   // printf("P: %d %d %d\n",location.lat,location.lng,location.alt);
   // printf("P(m) %f %f %f\n",position.x,position.y,position.z);
   // printf("v(m/s) %f %f %f\n",velocity_ef.x,velocity_ef.y,velocity_ef.z);
  //  printf("uvw_t(m/s) %f %f %f\n",velocity_t.x,velocity_t.y,velocity_t.z);
    
   // printf("H: %d %d %d\n",home.lat,home.lng,home.alt);
   // printf("O: %d %d %d\n",origin.lat,origin.lng,origin.alt);

  //  printf("roll %f %f  pitch %f %f  yaw  %f %f\n",static_cast<float>(pkt.g_packet.orientation_rpy_deg[0])*DEG_TO_RAD_DOUBLE,roll,
  //  static_cast<float>(pkt.g_packet.orientation_rpy_deg[1])*DEG_TO_RAD_DOUBLE,pitch,
  //  static_cast<float>(pkt.g_packet.orientation_rpy_deg[2])*DEG_TO_RAD_DOUBLE,yaw);
/*     ch[0] = pkt.g_packet.ch[0];
    ch[1] = pkt.g_packet.ch[1];
    ch[2] = pkt.g_packet.ch[2];
    ch[3] = pkt.g_packet.ch[3]; */
   
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.001 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    
    
    last_timestamp = pkt.g_packet.timestamp;

}

/*
  Drain remaining data on the socket to prevent phase lag.
 */
void FlightGear2::drain_sockets()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    errno = 0;
    do {
        received = socket_sitl.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);

}

/*
  update the FlightGear2 simulation by one time step
 */
void FlightGear2::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    //extrapolate_sensors(0.001);       // don't go past the nex);
    
    //update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
    drain_sockets();
    
}


}  // namespace SITL


#endif  // HAL_SIM_FlightGear2_ENABLED
