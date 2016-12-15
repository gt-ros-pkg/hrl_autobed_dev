#!/usr/bin/env python

import serial
import threading
import copy
import time
import signal
import sys

from numpy import nan

import rospy
import roslib
roslib.load_manifest('autobed_engine')


class SerialDriver():
    def __init__(self, num_values, port='/dev/ttyACM0', baudrate=9600, rate=1000.0):
        print "Serial driver: Starting reading encoder positions via serial."
        # signal.signal(signal.SIGINT, self.signal_handler) # Catch Ctrl-Cs - No longer needed.

        self.port_name = port
        self.baudrate = baudrate
        self.timeout = 0.1
        self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout)
        self.rate = rate

        self.num_values = num_values
        self.data = [nan] * self.num_values  # Locked by buffer_lock
        self.serial_data_buffer = ''  # Empty string used as a buffer for the data. Locked by serial_lock

        # Threading parameters
        self.buffer_lock = threading.RLock()
        self.serial_lock = threading.RLock()
        self.initThreading()

    ## Handler for Ctrl-C signals. Some of the ROS spin loops don't respond well to
    # Ctrl-C without this.
    def signal_handler(self, signal, frame):
        print 'Serial driver: Ctrl+C pressed - exiting.'
        sys.exit(0)

    def initThreading(self):
        self.serial_thread = threading.Thread(target=self.runSerialReader)
        self.serial_thread.daemon = True  # Run these as daemons so they'll die with Ctrl-C
        self.serial_thread.start()

    def getValue(self, value_idx):
        with self.buffer_lock:
            # data = [0.0] * self.num_values
            data = copy.copy(self.data)
            return data[value_idx]

    def getValues(self):
        with self.buffer_lock:
            # data = [0.0] * self.num_values
            data = copy.copy(self.data)
            return data

    def runSerialReader(self):
        while True:
            # print "running thread *******************************"
            time.sleep(
                (1.0 / self.rate))  # 10ms, can be removed but results in high CPU loads due to reading furiously.
            self.readFromSerial()

    def readFromSerial(self):
        # '''Read all data from the serial buffer, store the last block.
        #    Protocol is <num>,<num>,...,<num>\n
        # '''
        count = 0
        value_strings = None
        with self.serial_lock:
            # Block read everything.
            if self.serial_port.inWaiting() > 0:  # If we have bytes in the buffer
                self.serial_data_buffer += self.serial_port.read(
                    self.serial_port.inWaiting())  ## Append them to whatever was left last time

                buffer_elements = self.serial_data_buffer.split('\n')  # Break it into packets
                # print "Buffer elements: %s" %buffer_elements
                if len(buffer_elements) > 1:  # If we have at least two elements this means we got a full packet.
                    last_packet = buffer_elements[-2]  # This will always give us the last valid packet.
                    # print "last_packet is :", last_packet
                    try:
                        value_strings = last_packet.split(',')  # encoder_vals will be strings.
                    except:
                        print "Couldn't split value_strings on commas"
                        value_strings = None  # Garbage data, discard

                self.serial_data_buffer = buffer_elements[
                    -1]  # Update the data buffer with whatever the last frame element was.
                # Note that we added what we read to previous data to begin with, so this should be an update rather than append.
        # Release lock on serial port, though this should be the only routine using it.
        if value_strings != None and len(
                value_strings) == self.num_values:  # If we have at least as many data fields as our internal buffers, save them.
            with self.buffer_lock:  # Lock the buffer while we modify it.
                # print "got buffer lock"
                for i in range(self.num_values):
                    #          print "encoder_strings"
                    #          print encoder_strings
                    #          print "encoder_strings[i]"
                    #          print encoder_strings[i]
                    try:
                        data_i = float(value_strings[i])
                        #          print "data i"
                        #          print data_i
                        self.data[i] = data_i
                    except:
                        print "Couldn't convert data[%i]: %s to float" % (i, value_strings)
                        # Couldn't convert to floats, garbage data
                        pass

                        # print "end encoder data"
                        # print self.data


# If executed.
if __name__ == "__main__":
    dev = '/dev/ttyACM0'
    n = 1
    if len(sys.argv) > 1:
        dev = sys.argv[1]
    if len(sys.argv) > 2:
        n = int(sys.argv[2])

        # enc_driver = SerialDriver(n, dev)

    #  enc_driver = SerialDriver(2, port='/dev/ttyUSB0', baudrate=4800, rate=1000.0)
    enc_driver = SerialDriver(4, port='/dev/ttyACM0', baudrate=9600, rate=1000.0)
    # enc_driver = SerialDriver(1, port='/dev/ttyUSB0', baudrate=9600, rate=1000.0)
    drivers = []

    while True:
        time.sleep(0.01)
        print enc_driver.getValues()
