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
    if (!socket_sitl.bind(_flightgear_address, _flightgear_port)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", _flightgear_port);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    //_flightgear_address = address;
    //_flightgear_port = port_out;
    printf("Setting FlightGear interface to %s:%d \n", _flightgear_address, _flightgear_port);
}

/*
  decode and send servos
*/
void FlightGear::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    for (unsigned i = 0; i < 4; ++i)
    {
      pkt.motor_speed[i] = (input.servos[i]-1000) / 1000.0f;
    }
    socket_sitl.sendto(&pkt, sizeof(pkt), _flightgear_address, 5003);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void FlightGear::recv_fdm(const struct sitl_input &input)
{
    generic_packet pkt;
    
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_sitl.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch FlightGear reset
        if (get_wall_time_us() > last_wall_time_us + FLIGHTGEAR_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    printf("altitude %ld \n",  be64toh(pkt.position_xyz[2]));
    const double deltat = pkt.timestamp - last_timestamp;  // in seconds
    if (deltat < 0) {  // don't use old packet
        time_now_us += 1;
        return;
    }
    // get imu stuff
    accel_body = Vector3f(static_cast<float>(pkt.imu_linear_acceleration_xyz[0]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[1]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[2]));

    gyro = Vector3f(static_cast<float>(pkt.imu_angular_velocity_rpy[0]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[1]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[2]));

    // compute dcm from imu orientation
    Vector3f rpy = Vector3f(static_cast<float>(pkt.imu_orientation_rpy[0]),
                    static_cast<float>(pkt.imu_orientation_rpy[1]),
                    static_cast<float>(pkt.imu_orientation_rpy[2]));
    Quaternion quat;
    quat.from_euler(rpy);
    quat.rotation_matrix(dcm);

    velocity_ef = Vector3f(static_cast<float>(pkt.velocity_xyz[0]),
                           static_cast<float>(pkt.velocity_xyz[1]),
                           static_cast<float>(pkt.velocity_xyz[2]));

    position = Vector3d(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);
    position.xy() += origin.get_distance_NE_double(home);

    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    last_timestamp = pkt.timestamp;

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
