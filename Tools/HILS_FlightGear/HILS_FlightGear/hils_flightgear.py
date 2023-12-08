# This Python file uses the following encoding: utf-8
import sys
import os
import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
import serial
import serial.tools.list_ports
from mainwindow import Ui_MainWindow

# python -m PyQt5.uic.pyuic -x [변환대상.ui] -o [변환완료이름.py]


class HILS_FlightGear(QMainWindow):
    def __init__(self):
       QMainWindow.__init__(self)
       self.main_ui = Ui_MainWindow()
       self.main_ui.setupUi(self)
       self.find_port()
       self.show()


    def find_port(self):
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
            try:
                hwid.index('USB VID:PID=10C4:EA60 SER=0003')
                port1 = port
                print("{}: {} [{}]".format(port, desc, hwid))
                print("port1=",port1)
                self.main_ui.comboBox.addItem(port1)

            except:
                print("Not found")

            try:
                hwid.index('USB VID:PID=10C4:EA60 SER=0002')
                port2 = port
                print("{}: {} [{}]".format(port, desc, hwid))
                print("port2=",port2)
                self.main_ui.comboBox.addItem(port2)
            except:
                print("Not found")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = HILS_FlightGear()
    app.exec_()
