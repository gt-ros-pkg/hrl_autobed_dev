#!/usr/bin/env python

import serial

class AutobedDriver(object):
    cmdMap = {'headUP': 'A',
              'headDN': 'B',
              'bedUP': 'C',
              'bedDN': 'D',
              'legsUP': 'E',
              'legsDN': 'F'}

    def __init__(self, dev='/dev/ttyACM0', baudrate=9600):
        self.dev = dev;
        self.baudrate = baudrate;
        self.serial = serial.Serial(dev, baudrate)

    def send_command(self, cmd):
        if cmd in self.cmdMap:
            self.serial.flush()
            self.serial.write(self.cmdMap[cmd])
        else:
            raise Exception("[Autobed Driver] Received Unknown Command %s" %cmd)

    def close(self):
        self.serial.close()
