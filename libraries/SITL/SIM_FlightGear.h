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

#define NUM_ARRAY_DATA 18


typedef struct generic_packet {
  double timestamp;  // in seconds
  double lat_lon[2];
  float alt;
  float ch[4];
  float pilot_accel_swu_xyz[3];
  float orientation_rpy_deg[3];
  float pqr_rad[3];
  float speed_ned_fps[3];
  float rpm;
  //float uvw_body[3];
}Generic_packet;

typedef struct d_packet{
  uint64_t data64[3];
  uint32_t data32[18];
}D_packet;

typedef union u_packet{
  Generic_packet g_packet;
  D_packet dp;
}U_packet;

typedef union udp_in_packet{
  float serveo[8];
  uint32_t data[8];
}Udp_in_packet;

typedef struct fdm_data{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float phi;
    float theta;
    float psi;
    float p;
    float q;
    float r;
}Fdm_data;

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

      void recv_fdm(const struct sitl_input &input);
      void send_servos(const struct sitl_input &input);
      void drain_sockets();

      double last_timestamp;

      SocketAPM socket_sitl;
      const char *_flightgear_address = "127.0.0.1";
      int _flightgear_port = 9002;
      static const uint64_t FLIGHTGEAR_TIMEOUT_US = 5000000;

      float ch[4]={0.0,0.0,0.0,0.0};
      int count = 0;
  };

}  // namespace SITL


#endif  // HAL_SIM_FLIHGTGEAR_ENABLED
