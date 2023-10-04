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
  simulator connector for ardupilot version of FlightGear
*/

#include "SIM_FlightGear.h"

#if HAL_SIM_FLIHGTGEAR_ENABLED

#include <stdio.h>
#include <errno.h>

namespace SITL {

FlightGear::FlightGear(const char *frame_str) :
    Aircraft(frame_str),
    last_timestamp(0),
    socket_sitl{true}
{
    fprintf(stdout, "Starting SITL FlightGear\n");
}

/*
  Create and set in/out socket
*/
void FlightGear::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // FlightGear keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!socket_sitl.bind("0.0.0.0", port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", port_in);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    _flightgear_address = address;
    _flightgear_port = port_out;
    printf("Setting FlightGear interface to %s:%d \n", _flightgear_address, _flightgear_port);
}

/*
  decode and send servos
*/
void FlightGear::send_servos(const struct sitl_input &input)
{
   udp_in_packet pkt;

    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
 //   for (unsigned i = 0; i < 4; ++i)
   // {
      //double temp = (input.servos[i]-1000) / 1000.0f;
  //    double temp = ((double) i + 1.0)/10/0;
  //    pkt.data[i] = be64toh(temp);
  //  }
    pkt.serveo[0] = (input.servos[0]-1500) / 500.0f *1.0;
    pkt.serveo[1] = -(input.servos[1]-1500) / 500.0f *1.0;
    pkt.serveo[2] = (input.servos[2]-1000) / 1000.0f;
    pkt.serveo[3] = (input.servos[3]-1500) / 500.0f * 0.2;
    pkt.serveo[4] = (input.servos[2]-1000) / 1000.0f;
    
    uint32_t data[5];
    data[0] = __bswap_32(pkt.data[0]);
    data[1] = __bswap_32(pkt.data[1]);
    data[2] = __bswap_32(pkt.data[2]);
    data[3] = __bswap_32(pkt.data[3]);
    data[4] = __bswap_32(pkt.data[4]);
    socket_sitl.sendto(&data, sizeof(data), _flightgear_address, 9999);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void FlightGear::recv_fdm(const struct sitl_input &input)
{
    U_packet pkt;
    Generic_uint_packet fdm_data;
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_sitl.recv(&fdm_data.data32, sizeof(Generic_uint_packet), 100) != sizeof(Generic_uint_packet)) {
        
       // send_servos(input);
        // Reset the timestamp after a long disconnection, also catch FlightGear reset
        if (get_wall_time_us() > last_wall_time_us + FLIGHTGEAR_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    for (long unsigned int i=0; i < sizeof(fdm_data.data32)/4; i++){
        pkt.uint_data.data32[i] = be32toh(fdm_data.data32[i]);
    }
    for (long unsigned int i=0; i < sizeof(fdm_data.data64)/8; i++){
        pkt.uint_data.data64[i] = be64toh(fdm_data.data64[i]);
    }
    printf("time stamp %f \n",  pkt.g_packet.timestamp);
    const double deltat = pkt.g_packet.timestamp - last_timestamp;  // in seconds
    if (deltat < 0) {  // don't use old packet
        time_now_us += 1;
        return;
    }

    accel_body = Vector3f(pkt.g_packet.imu_linear_acceleration_xyz[0]* FEET_TO_METERS,
                         -pkt.g_packet.imu_linear_acceleration_xyz[1]* FEET_TO_METERS,
                          pkt.g_packet.imu_linear_acceleration_xyz[2]* FEET_TO_METERS);


     gyro = Vector3f(pkt.g_packet.imu_angular_velocity_rpy[0]*DEG_TO_RAD_DOUBLE,
                     pkt.g_packet.imu_angular_velocity_rpy[1]*DEG_TO_RAD_DOUBLE,
                     pkt.g_packet.imu_angular_velocity_rpy[2]*DEG_TO_RAD_DOUBLE );

    velocity_ef = Vector3f(pkt.g_packet.velocity_xyz[0] * FEET_TO_METERS, 
                            pkt.g_packet.velocity_xyz[1] * FEET_TO_METERS,
                            pkt.g_packet.velocity_xyz[2]  * FEET_TO_METERS);

    // compute dcm from imu orientation
    

    Quaternion quat;
    quat.from_euler(pkt.g_packet.imu_orientation_rpy[0]*DEG_TO_RAD_DOUBLE,
                    pkt.g_packet.imu_orientation_rpy[1]*DEG_TO_RAD_DOUBLE,
                    pkt.g_packet.imu_orientation_rpy[2]*DEG_TO_RAD_DOUBLE);

    quat.rotation_matrix(dcm);

  printf("a:%.2f %.2f %f g:%.2f %.2f %.2f A:%.2f %.2f %.2f\n",accel_body.x,accel_body.y,accel_body.z,
                                                gyro.x,gyro.y,gyro.z,
                                                pkt.g_packet.imu_orientation_rpy[0],pkt.g_packet.imu_orientation_rpy[1],
                                                pkt.g_packet.imu_orientation_rpy[2]);

    Location loc_current = Location(static_cast<int32_t>(pkt.g_packet.position_xyz[0]*1.0e7),
                            static_cast<int32_t>(pkt.g_packet.position_xyz[1]*1.0e7),
                            static_cast<int32_t>(pkt.g_packet.position_xyz[2]*30.48), Location::AltFrame::ABOVE_HOME);

    position = origin.get_distance_NED_double(loc_current);

    printf("%f %f %f\n",pkt.g_packet.position_xyz[0],pkt.g_packet.position_xyz[1],pkt.g_packet.position_xyz[2]);

    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    last_timestamp = pkt.g_packet.timestamp;

}

/*
  Drain remaining data on the socket to prevent phase lag.
 */
void FlightGear::drain_sockets()
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
  update the FlightGear simulation by one time step
 */
void FlightGear::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
    drain_sockets();
}

}  // namespace SITL


#endif  // HAL_SIM_FLIGHTGEAR_ENABLED
