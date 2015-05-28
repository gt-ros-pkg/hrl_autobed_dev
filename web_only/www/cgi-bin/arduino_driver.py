#!/usr/bin/env python

import serial

cmdMap = {'headUP': 'A',
          'headDN': 'B',
          'bedUP': 'C',
          'bedDN': 'D',
          'legsUP': 'E',
          'legsDN': 'F'}

class AutobedDriver(object)
    def __init(self, dev='/dev/ttyACM0', baudrate=9600):
        self.dev = dev;
        self.baudrate = baurdate;
        self.serial = serial.Serial(dev, baudrate)

    def send_command(self, cmd):
        if cmd in cmdMap:
            self.serial.write(cmdMap[cmd])
        else:
            raise Exception("[Autobed Driver] Received Unknown Command %s" %cmd)
