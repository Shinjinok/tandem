#HILS for run ./fg_ch47_view.sh
# MAVProxy    mavproxy.py --master=/dev/ttyACM0 --console --map
#virtual serial port  socat -d -d pty,raw,echo=0 pty,raw,echo=0
#usage:
#PARAMS:
# param set AHRS_EKF_TYPE 11
# param set EAHRS_TYPE 3
# param set EAHRS_RATE 50
# param set SERIAL2_PROTOCOL 36  
# param set SERIAL2_BAUD 460 460800
# param set GPS_TYPE 21 <--GPS_TYPE_EXTERNAL_AHRS = 21,
# param set FS_OPTIONS 0  <--Failsafe disable
# param set DISARM_DELAY 0
# param set ARMING_CHECk 0
# param set H_SW_TYPE 1
# param set BRD_SAFETY_DEFLT 0
# param set ATC_RATE_Y_MAX 10
# param set INS_USE2 0
# param set INS_ENABLE_MASK 1
# param set INS_FAST_SAMPLE 1
# param set RK3_SRC1_POSZ 3
# param set ATC_HOVR_ROL_TRM 0
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
import glob
import errno
import numpy as np # Scientific computing library for Python
from scipy.spatial.transform import Rotation as R
import time
import serial.tools.list_ports
import subprocess

deg2rad = 0.0174533

class GPS:
  lat = None
  lon = None
  fix_type = 5
class ATT:
  roll = None
  pitch = None
  yaw = None
  

# RPY/Euler angles to Rotation Vector
def rotation_matrix(roll_deg, pitch_deg, yaw_deg):

  cr = np.cos(roll_deg * np.pi / 180)
  sr = np.sin(roll_deg * np.pi / 180)
  cp = np.cos(pitch_deg * np.pi / 180)
  sp = np.sin(pitch_deg * np.pi / 180)
  cy = np.cos(yaw_deg * np.pi / 180)
  sy = np.sin(yaw_deg * np.pi / 180)

  etb_yaw = np.array([
      [cy, cy, 0],
      [-sy,  cy, 0],
      [ 0,   0, 1]])
  etb_pitch = np.array([
      [ cp, 0, -sp],
      [ 0, 1,  0],
      [sp, 0, cp]])
  etb_roll = np.array([
      [1,  0,  0],
      [0, cr, sr],
      [0, -sr,  cr]])
  # R = RzRyRx
  etb_rotMat = np.dot(etb_roll, np.dot(etb_pitch, etb_yaw))
  return etb_rotMat




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

          
rsp_serial1 = False
rsp_serial2 = False

if __name__ == '__main__':
  
  
  #subprocess.call(['xterm', '-e', 'mavproxy.py --master=/dev/ttyACM1 --console --map'])
  #subprocess.call(['xterm', '-e', 'mavproxy.py --master=/dev/ttyACM3 --console --map'])
  
  
  ports = serial.tools.list_ports.comports()

  for port, desc, hwid in sorted(ports):
      try:
        hwid.index('USB VID:PID=10C4:EA60 SER=0003')
        port1 = port
        print("{}: {} [{}]".format(port, desc, hwid))
        print("port1=",port1)
      except:
        print("Not found")
      try:
        hwid.index('USB VID:PID=10C4:EA60 SER=0002')
        port2 = port
        print("{}: {} [{}]".format(port, desc, hwid))
        print("port2=",port2)
      except:
        print("Not found")  
        
    
  
        
  udp = udp_socket("127.0.0.1:9003:9002")#ip, in ,out
  try:
    serial1 = usbSerial(port1)
  except:
    serial1  = None
    print("no serial 1")
   
  try:  
    serial2 =usbSerial(port2)
  except:
    serial2 = None
    print("no serial 2")
    
  gps = GPS()
  att = ATT()
  #1/1/1970~1/6/1980 522weeks 3days tokyo utc+9 and correct 18secs
  fix_time = -522*7*24*60*60*1000 - 3*24*60*60*1000  + 9*60*60*1000 + 18*1000
  while True:
    
    if udp.udp_data_in_flag:
      udp.udp_data_in_flag = False
      lat = udp.udp_data_in[1]
      lon = udp.udp_data_in[2]
      

      gps.lat = udp.udp_data_in[1] *1e7
      gps.lon = udp.udp_data_in[2] *1e7
      att.roll = udp.udp_data_in[13]*deg2rad
      att.pitch = udp.udp_data_in[14]*deg2rad
      att.yaw = udp.udp_data_in[15]*deg2rad
      m = mavextra.expected_mag(gps,att)
      baro = float(udp.udp_data_in[16]*3386.39) # Convert millibar to pascals
      gps_time= time.time()*1000 + fix_time

      d = pack('<3B3d18f',0xFE, 0xBB, 0xAA,
               gps_time,
               udp.udp_data_in[1],#lat
               udp.udp_data_in[2],#lat
               udp.udp_data_in[3],#alt
               udp.udp_data_in[4],udp.udp_data_in[5],udp.udp_data_in[6],# pqr
               udp.udp_data_in[7]*0.3048,udp.udp_data_in[8]*0.3048,udp.udp_data_in[9]*0.3048, #acc x y z
               udp.udp_data_in[10]*0.3048,udp.udp_data_in[11]*0.3048,udp.udp_data_in[12]*0.3048, #speed_ned
               udp.udp_data_in[13]*deg2rad,udp.udp_data_in[14]*deg2rad,udp.udp_data_in[15]*deg2rad,#roll pitch yaw
               baro, #pressure pascal 
               udp.udp_data_in[17],#rpm
               m.x, m.y, m.z)

      if serial1 != None:
        serial1.write(d)
      if serial2 != None: 
        serial2.write(d)
        
      time.sleep(0.01)
      
    if serial1 != None :
      if serial1.serial_data_in_flag:
        serial1.serial_data_in_flag = False
        clock = time.clock_gettime(0)
        deltat = clock - serial1.delta_read_time 
        serial1.delta_read_time = clock
        a = str(serial1.serial_data_in).split(':')
        print("primary   :",a,rsp_serial1)
        if len(a) == 10:
          send_data = []
          send_data.append(float((float(a[1] ) - 1500.0) / 250.0))
          send_data.append(float((float(a[2] ) - 1500.0) / -250.0))
          send_data.append(float((2000.0 - float(a[3] )) / 1000.0))
          send_data.append(float((float(a[4] ) - 1500.0) / 500.0))
          send_pack= pack('>5f',send_data[0],send_data[1] ,send_data[2] ,send_data[3] ,send_data[2])
          udp.write(send_pack)
          rsp_serial1 = True
          rsp_serial2 = False 
        else :
          rsp_serial1 = False  
        
    if serial2 != None :
      if serial2.serial_data_in_flag:
        serial2.serial_data_in_flag = False
        a = str(serial2.serial_data_in).split(':')
        print("secondary :",a,rsp_serial2)
        if len(a) == 10:
          send_data = []
          send_data.append(float((float(a[1] ) - 1500.0) / 250.0))
          send_data.append(float((float(a[2] ) - 1500.0) / -250.0))
          send_data.append(float((2000.0 - float(a[3] )) / 1000.0))
          send_data.append(float((float(a[4] ) - 1500.0) / 500.0))
          send_pack= pack('>5f',send_data[0],send_data[1] ,send_data[2] ,send_data[3] ,send_data[2])
          
          if rsp_serial1 == False :
            udp.write(send_pack)
            rsp_serial2 = True
        else :
          rsp_serial2 = False        
     
     


