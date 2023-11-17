#HILS for run ./fg_ch47_view.sh
# MAVProxy    mavproxy.py --master=/dev/ttyACM0 --console --map
#virtual serial port  socat -d -d pty,raw,echo=0 pty,raw,echo=0
#usage:
#PARAMS:
# param set AHRS_EKF_TYPE 11
# param set EAHRS_TYPE 3
# param set SERIAL1_PROTOCOL 36  
# param set SERIAL1_BAUD 480 480600
# param set GPS_TYPE 21 <--GPS_TYPE_EXTERNAL_AHRS = 21,
# param set FS_OPTIONS 0  <--Failsafe disable
# param set DISARM_DELAY 0
# param set ARMING_CHECk 0
# RC_OPTION ignore rc receiver

'''
generate field tables from IGRF13. Note that this requires python3
'''
from pymavlink import mavextra
import socket
from struct import pack, unpack,calcsize
import serial
import threading
import time
import sys
import errno
import numpy as np # Scientific computing library for Python
from scipy.spatial.transform import Rotation as R


PORT = '/dev/ttyUSB0'
#virtual serial port
#PORT = '/dev/pts/3' 
PORT2= '/dev/pts/4'

deg2rad = 0.0174533

# RPY/Euler angles to Rotation Vector
def rotation_matrix(theta1, theta2, theta3):
  """
  input
      theta1, theta2, theta3 = rotation angles in rotation order (degrees)
      oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
  output
      3x3 rotation matrix (numpy array)
  """
  c1 = np.cos(theta1 * np.pi / 180)
  s1 = np.sin(theta1 * np.pi / 180)
  c2 = np.cos(theta2 * np.pi / 180)
  s2 = np.sin(theta2 * np.pi / 180)
  c3 = np.cos(theta3 * np.pi / 180)
  s3 = np.sin(theta3 * np.pi / 180)
  matrix=np.array([[c2, -c3*s2, s2*s3],
            [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
            [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]])
  return matrix

def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]])
    Ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    # R = RzRyRx
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat




class usbSerial(object):
  def __init__(self, port):
    try:
      self.seri = serial.Serial(port, baudrate=480600, timeout=1)
    except Exception as e:
      print("error open serial port: " + str(e))
      exit()
    self.serial_data_in = None
    self.serial_data_in_flag  = False
    self.udp_raw_data_in =None
    self.lock = threading.Lock()
    self.delta_read_time = 0
    t = threading.Thread(target=self.listen)
    t.start()

  def listen(self):
    while True:
      #time.sleep(0.1)
      try:
        self.serial_data_in = self.seri.readline()
        self.serial_data_in_flag = True
      except:
        print("abnomal\n")
        self.serial_data_in_flag = False

          


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
      
      self.udp_data_in = unpack('>3d15f',data_udp)
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
      lat = udp.udp_data_in[1]
      lon = udp.udp_data_in[2]
      m = mavextra.expected_earth_field_lat_lon(lat, lon)
      print("{%f, %f, {%.3f, %.3f, %.3f}}," % (lat, lon, m.x, m.y, m.z))
      rot = rotation_matrix(-udp.udp_data_in[13], -udp.udp_data_in[14], -udp.udp_data_in[15])
      v=[]
      v.append(m.x)
      v.append(m.y)
      v.append(m.z)
      rm = np.dot(rot, v)
      
      print("rm",rm,"---")
      d = pack('<3B3d18f',0xFE, 0xBB, 0xAA,
               udp.udp_data_in[0],
               udp.udp_data_in[1],#lat
               udp.udp_data_in[2],#lat
               udp.udp_data_in[3],#alt
               udp.udp_data_in[4],udp.udp_data_in[5],udp.udp_data_in[6],# pqr
               udp.udp_data_in[7]*0.3048,udp.udp_data_in[8]*0.3048,udp.udp_data_in[9]*0.3048, #acc x y z
               udp.udp_data_in[10]*0.3048,udp.udp_data_in[11]*0.3048,udp.udp_data_in[12]*0.3048, #speed_ned
               udp.udp_data_in[13]*deg2rad,udp.udp_data_in[14]*deg2rad,udp.udp_data_in[15]*deg2rad,#roll pitch yaw
               udp.udp_data_in[16]*33.8637526, #pressure mbar
               udp.udp_data_in[17],#rpm
               rm[0],rm[1],rm[2]) 

      
      seri.write(d)
      print("write to serial 1")
      print(udp.udp_data_in[16]*33.8637526)
      time.sleep(0.1)
      
    if seri.serial_data_in_flag:
      seri.serial_data_in_flag = False
      clock = time.clock_gettime(0)
      deltat = clock - seri.delta_read_time 
      seri.delta_read_time = clock
      print("serial1 received data", seri.serial_data_in , deltat)
      
      a = str(seri.serial_data_in).split(':')
      print(a)
      send_data = []
      for i in range(8):
        send_data.append((float(a[i+1] ) - 1500.0) / 500.0)
      send_data[2] = (2000.0 - float(a[3] )) / 1000.0
      #send_data[2] = 0.4
      #send_data= pack('>5f',0.1,0.2,0.3,0.4,0.5)
      send_pack= pack('>5f',send_data[0],-send_data[1] ,send_data[2] ,send_data[3] ,send_data[2])
      print(send_data)
      udp.write(send_pack)
     
     


