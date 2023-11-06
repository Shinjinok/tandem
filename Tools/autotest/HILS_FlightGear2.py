#HILS for run ./fg_ch47_view.sh
#virtual serial port  socat -d -d pty,raw,echo=0 pty,raw,echo=0
#usage:
#PARAMS:
# param set AHRS_EKF_TYPE 11
# param set EAHRS_TYPE 3
# param set SERIAL1_PROTOCOL 36
# param set SERIAL1_BAUD 480


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



PORT = '/dev/ttyUSB0'
#virtual serial port
#PORT = '/dev/pts/3' 
PORT2= '/dev/pts/4'

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
    while True:
      data_udp ,addr= self.port.recvfrom(1024)
      self.udp_raw_data_in = data_udp
      self.udp_data_in = unpack('>dddffffffffffffffffff',data_udp)
      self.udp_data_in_flag = True

          


if __name__ == '__main__':
  # Make sure all log messages show up
  udp = udp_socket("127.0.0.1:9003:9002")#ip, in ,out
  seri = usbSerial(PORT)
  seri2 = None#usbSerial(PORT2)

  count = 1
  head = pack('<3B',0xFE, 0xBB, 0xAA)

 
  while True:
    
    """     seri.write(bytes("test serial tx rx\n",encoding='ascii'))
    print("write to serial 1") """
    time.sleep(1)
    if udp.udp_data_in_flag:
      udp.udp_data_in_flag = False
      serial_out = [head, udp.udp_raw_data_in]
      
      print(head)
      #seri.write(bytes(head))
      print(len(udp.udp_raw_data_in))
      seri.write(bytes(head)+bytes(udp.udp_raw_data_in))
      seri.write(bytes(udp.udp_raw_data_in))
      print(bytes(head)+bytes(udp.udp_raw_data_in))
      print(bytes(udp.udp_raw_data_in))

      print("write to serial 1")

     # print("write serial1\n")
      
    if seri.serial_data_in_flag:
      seri.serial_data_in_flag = False
      print("serial1 received data", seri.serial_data_in)
     
     


