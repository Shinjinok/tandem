#HILS for run ./fg_ch47_view.sh
#virtual serial port  socat -d -d pty,raw,echo=0 pty,raw,echo=0

import socket
from struct import pack, unpack
import serial
import threading
import time
import sys
import errno



#PORT = '/dev/ttyUSB0'
#virtual serial port
PORT = '/dev/pts/3' 
PORT2= '/dev/pts/4'

class usbSerial(object):
  def __init__(self, port):
    try:
      self.seri = serial.Serial(port, baudrate=115200, timeout=1)
    except Exception as e:
      print("error open serial port: " + str(e))
      exit()
    self.serial_data_in = None
    self.serial_data_in_flag  = False
    self.lock = threading.Lock()
    t = threading.Thread(target=self.listen)
    t.start()

  def listen(self):
    while True:
      self.serial_data_in = self.seri.readline()
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

    self.ser = serial.Serial(PORT, baudrate=115200, timeout=1)

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
      self.udp_data_in = unpack('>dddffffffffffffffffff',data_udp)
      self.udp_data_in_flag = True

          


if __name__ == '__main__':
  # Make sure all log messages show up
  udp = udp_socket("127.0.0.1:9003:9002")#ip, in ,out
  seri = usbSerial(PORT)
  seri2 = usbSerial(PORT2)

  count = 1

 
  while True:
    if udp.udp_data_in_flag:
      udp.udp_data_in_flag = False
      out = str(udp.udp_data_in[0])+"\n"
      print("udp received data",udp.udp_data_in[0] )
      seri.write(bytes(out,encoding='ascii'))
      print("write to serial 1")
     # print("write serial1\n")
    
    if seri2.serial_data_in_flag:
      seri2.serial_data_in_flag = False
      #print(seri2.serial_data_in)
      print("serial2 received data ",seri2.serial_data_in)

      if count == 1:
        count = 0
        strdata=""
        ch = [0.0,0.0,0.0,1.0,0.0]
        
        for n in range(len(ch)-1):
          strdata += str(ch[n])+":"
        strdata += str(ch[n+1])  

      else:
        count =1
        strdata=""
        ch = [0.0,0.0,0.0,-1.0,0.0]

        for n in range(len(ch)-1):
          strdata += str(ch[n])+":"
        strdata += str(ch[n+1])    
      strdata += "\n"
      
      seri2.write(bytes(strdata,encoding='utf-8'))
      print("write data serial 2", strdata)
      
    if seri.serial_data_in_flag:
      seri.serial_data_in_flag = False
      print("serial1 received data", seri.serial_data_in)
      deco = seri.serial_data_in.decode('utf-8')
      #print(deco)
      deco2 = deco.splitlines()
      #print(deco2)
      a = deco2[0].split(':')
      #print(a)
      b=[]
      for n in range(len(a)):
        b.append(float(a[n]))
      print("send serial 1 to udp", b)
      out_data = pack('>5f',b[0],b[1],b[2],b[3],b[4])
      udp.write(out_data)

