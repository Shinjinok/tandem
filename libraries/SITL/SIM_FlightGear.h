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
  simulator connection for ardupilot version of FlightGear
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_FLIHGTGEAR_ENABLED
#define HAL_SIM_FLIHGTGEAR_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_FLIHGTGEAR_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket.h>

namespace SITL {

/*
  FlightGear simulator
 */
class FlightGear : public Aircraft {
public:
    FlightGear(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new FlightGear(frame_str);
    }

    /*  Create and set in/out socket for FlightGear simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    /*
      packet sent to FlightGear
     */
    struct servo_packet {
      // size matches sitl_input upstream
      float motor_speed[16];
    };

    /*
      reply packet sent from FlightGear to ArduPilot
     */
    struct fdm_packet {
      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
    };

    struct generic_packet {
      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_rpy[3];
      double velocity_xyz[3];
      double position_xyz[3];
    };


    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void drain_sockets();

    double last_timestamp;

    SocketAPM socket_sitl;
    const char *_flightgear_address = "127.0.0.1";
    int _flightgear_port = 5001;
    static const uint64_t FLIGHTGEAR_TIMEOUT_US = 5000000;
};

}  // namespace SITL


#endif  // HAL_SIM_FLIHGTGEAR_ENABLED
