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

    pkt.serveo[0] = (input.servos[0]-1500) / 500.0f *2.0;
    pkt.serveo[1] = -(input.servos[1]-1500) / 500.0f *2.0;
    pkt.serveo[2] = (2000- input.servos[2]) / 1000.0f;
    pkt.serveo[3] = (input.servos[3]-1500) / 500.0f *2.0;
    pkt.serveo[4] = (input.servos[2]-1000) / 1000.0f;
    

   /*pkt.serveo[0] = (ch[0] -0.5)*2.0;
   pkt.serveo[1] = (-ch[1] +0.5)*2.0;
   pkt.serveo[2] = 0.0;
   pkt.serveo[3] = (ch[3]-0.5)*2.0;
   pkt.serveo[4] = ch[2];*/

    uint64_t data[5];
    data[0] = __bswap_64(pkt.data[0]);
    data[1] = __bswap_64(pkt.data[1]);
    data[2] = __bswap_64(pkt.data[2]);
    data[3] = __bswap_64(pkt.data[3]);
    data[4] = __bswap_64(pkt.data[4]);
    socket_sitl.sendto(&data, sizeof(data), _flightgear_address, 9999);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void FlightGear::recv_fdm(const struct sitl_input &input)
{
    U_packet pkt;
    uint64_t fdm_data[NUM_ARRAY_DATA];
    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_sitl.recv(&fdm_data, sizeof(fdm_data), 100) != sizeof(fdm_data)) {
        
        //send_servos(input);
        // Reset the timestamp after a long disconnection, also catch FlightGear reset
        if (get_wall_time_us() > last_wall_time_us + FLIGHTGEAR_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    for (long unsigned int i=0; i < NUM_ARRAY_DATA; i++){
        pkt.data64[i] = __bswap_64(fdm_data[i]);
    }

    const double deltat = pkt.g_packet.timestamp - last_timestamp;  // in seconds

    if (deltat < 0) {  // don't use old packet
        time_now_us += 1;
        return;
    }

    accel_body = Vector3f(pkt.g_packet.pilot_accel_nwu_xyz[0] * FEET_TO_METERS,
                          -pkt.g_packet.pilot_accel_nwu_xyz[1] * FEET_TO_METERS,
                          pkt.g_packet.pilot_accel_nwu_xyz[2] * FEET_TO_METERS);

    
    Fdm_data new_data;                         
    new_data.phi = pkt.g_packet.orientation_rpy_deg[0]*DEG_TO_RAD_DOUBLE;
    new_data.theta = pkt.g_packet.orientation_rpy_deg[1]*DEG_TO_RAD_DOUBLE;
    new_data.psi = pkt.g_packet.orientation_rpy_deg[2]*DEG_TO_RAD_DOUBLE;
    // compute dcm from imu orientation
    dcm.from_euler(new_data.theta,new_data.phi,new_data.psi);
    //Vector3f rad = Vector3f(new_data.phi,new_data.theta, new_data.psi);
    //Quaternion quat;
    //quat.from_axis_angle(rad);
    //quat.rotation_matrix(dcm);
    Vector3f g = Vector3f(0.0f, 0.0f, -GRAVITY_MSS);
    Vector3f acc = dcm*g;
   
    Location current;
    current.lat = pkt.g_packet.position_la_lon_alt[0] * 1.0e7;
    current.lng = pkt.g_packet.position_la_lon_alt[1] * 1.0e7;
    current.alt = pkt.g_packet.position_la_lon_alt[2]* FEET_TO_METERS * 100.0f + home.alt;

    position = origin.get_distance_NED_double(current);

    new_data.x = position.x;
    new_data.y = position.y;
    new_data.z = position.z;


    new_data.p = (new_data.phi - old_data.phi) / deltat;
    new_data.q = (new_data.theta - old_data.theta) / deltat;
    new_data.r = (new_data.psi - old_data.psi) / deltat;
    //gyro = Vector3f(new_data.p, new_data.q, new_data.r);

    double p, q, r;
    SIM::convert_body_frame(degrees(new_data.phi ), degrees(new_data.theta),
                             degrees(new_data.p), degrees(new_data.q ), degrees(new_data.r ),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    new_data.vx = (new_data.x - old_data.x) /deltat;
    new_data.vy = (new_data.y - old_data.y) /deltat;
    new_data.vz = (new_data.z - old_data.z) /deltat;
    velocity_ef = Vector3f(new_data.vx, new_data.vy, new_data.vz);


    old_data = new_data;
    printf("deltat %f\n",deltat);
    printf("A: %3.3f %3.3f %3.3f G: %3.3f %3.3f %3.3f V:%f %f %f\n",accel_body.x,accel_body.y,accel_body.z,
                                                gyro.x,gyro.y,gyro.z, new_data.vx, new_data.vy,new_data.vz);
    printf("a: %3.3f %3.3f %3.3f \n",acc.x,acc.y,acc.z);
    
    printf("P: %f %f %f\n",new_data.x,new_data.y,new_data.z);
    ch[0] = pkt.g_packet.ch[0];
    ch[1] = pkt.g_packet.ch[1];
    ch[2] = pkt.g_packet.ch[2];
    ch[3] = pkt.g_packet.ch[3];
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
    //extrapolate_sensors(delta_time);
    
    update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
    drain_sockets();
}

}  // namespace SITL


#endif  // HAL_SIM_FLIGHTGEAR_ENABLED
