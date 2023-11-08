#HILS for run ./fg_ch47_view.sh
#virtual serial port  socat -d -d pty,raw,echo=0 pty,raw,echo=0
#usage:
#PARAMS:
# param set AHRS_EKF_TYPE 11
# param set EAHRS_TYPE 3
# param set SERIAL1_PROTOCOL 36
# param set SERIAL1_BAUD 480
# param set GPS_TYPE 21 <--GPS_TYPE_EXTERNAL_AHRS = 21,


"""
struct PACKED Generic_packet {
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
};
"""

import socket
from struct import pack, unpack,calcsize
import serial
import threading
import time
import sys
import errno
import numpy as np # Scientific computing library for Python


PORT = '/dev/ttyUSB0'
#virtual serial port
#PORT = '/dev/pts/3' 
PORT2= '/dev/pts/4'

deg2rad = 0.0174533

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


class usbSerial(object):
  def __init__(self, port):
    try:
      self.seri = serial.Serial(port, baudrate=460800, timeout=1)
    except Exception as e:
      print("error open serial port: " + str(e))
      exit()
    self.serial_data_in = None
    self.serial_data_in_flag  = False
    self.udp_raw_data_in =None
    self.lock = threading.Lock()
    t = threading.Thread(target=self.listen)
    t.start()

  def listen(self):
    while True:
      time.sleep(0.1)
      self.serial_data_in = self.seri.readline()
      if len(self.serial_data_in) > 0 :
          self.serial_data_in_flag = True


  def write(self, data):
      self.seri.write(data)

class udp_socket(object):
  """A UDP socket."""
  def __init__(self, device):
    a = device.split(':')
    print(a)

    self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.port.bind((a[0], int(a[1])))
    self.destination_addr = (a[0], int(a[2]))
    self.udp_data_in = None
    self.udp_data_in_flag = False
    self.udp_raw_data_in = None

    #self.ser = serial.Serial(PORT, baudrate=115200, timeout=1)

    t = threading.Thread(target=self.listen_clients)
    t.start()

  def write(self, buf):
    try:
        self.port.sendto(buf, self.destination_addr)

    except socket.error:
        pass
      
  def listen_clients(self):
    print("start thread\n")
    time.sleep(0.1)
    while True:
      data_udp ,addr= self.port.recvfrom(1024)
      
      self.udp_data_in = unpack('>3d14f',data_udp)
      self.udp_raw_data_in = data_udp
      self.udp_data_in_flag = True

          


if __name__ == '__main__':
  # Make sure all log messages show up
  udp = udp_socket("127.0.0.1:9003:9002")#ip, in ,out
  seri = usbSerial(PORT)
  seri2 = None#usbSerial(PORT2)

 
  while True:
    
    if udp.udp_data_in_flag:
      udp.udp_data_in_flag = False
 
      d = pack('<3B3d13f',0xFE, 0xBB, 0xAA,
               udp.udp_data_in[0],
               udp.udp_data_in[1],udp.udp_data_in[2],#lat lon
               udp.udp_data_in[3],#alt
               udp.udp_data_in[4],udp.udp_data_in[5],udp.udp_data_in[6],# pqr
               udp.udp_data_in[7]*0.3048,udp.udp_data_in[8]*0.3048,udp.udp_data_in[9]*0.3048, #acc x y z
               udp.udp_data_in[10]*0.3048,udp.udp_data_in[11]*0.3048,udp.udp_data_in[12]*0.3048, #speed_ned
               udp.udp_data_in[13]*deg2rad,udp.udp_data_in[14]*deg2rad,udp.udp_data_in[15]*deg2rad) #roll pitch yaw

      seri.write(d)
      print("write to serial 1")
      time.sleep(0.1)
    if seri.serial_data_in_flag:
      seri.serial_data_in_flag = False
      print("serial1 received data", seri.serial_data_in)
     
     


