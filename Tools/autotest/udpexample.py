import socket
import struct
import time
import numpy as np
from dataclasses import dataclass

@dataclass
class Control:
    aileron:float = None
    elevator:float = None
    rudder:float = None
    throttle:float = None
    
control = Control()
control.aileron = 1
control.elevator = 1
control.rudder = 1
control.throttle = 1

UDP_IP = "127.0.0.1"
UDP_PORT_IN= 5503
UDP_PORT_OUT= 5001

sock_out = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_in = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_out.bind((UDP_IP, UDP_PORT_OUT))
sock_in.bind((UDP_IP, UDP_PORT_IN))
i=1
while True:
    data, addr = sock_out.recvfrom(1024) # buffer size is 1024 bytes
    if i==0.5123 :
        data_pack = struct.pack('>dddd',i,i,i,i)
        i=-0.5123
    else :
        data_pack = struct.pack('>dddd',i,i,i,i)
        i=0.5123
    sock_in.sendto(data_pack,(UDP_IP, 5003))
    print(data_pack)
    time.sleep(1)
    print("received message: %s" % data)
    b=struct.unpack('>dddddddddddddddd', data)
    print(b)

