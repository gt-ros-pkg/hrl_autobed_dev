#!/usr/bin/env python

import socket
import RPi.GPIO as GPIO
from threading import Timer, Lock
import atexit

#socket parameters
HOST = 'localhost'  # only listen to loopback interface (only connections from this machine)
PORT = 828  # arbitrary, non-reserved, privledged port (the HRL building address)

# list of GPIO pin numbers (per Broadcom scheme) used to control autobed;
pins = [17, 18, 27, 22, 23, 24]
timers = {}
locks = {}

# Register cleanup handler to shutdown all pins and stop timers, then cleanup GPIO settings
@atexit.register
def cleanup():
    for pin in pins:
        with locks[pin]:
            GPIO.output(pin, GPIO.LOW)  # Set pins low
            if not timers[pin] is None:
		    timers[pin].cancel()  # Stop timers
		    timers[pin] = None
    GPIO.cleanup()
    print "[GPIO Daemon] Cleanup of Autobed GPIO configuration complete."

# Setup GPIO settings, make timer and locks for each pin, set each pin to OUTPUT, LOW (off).
GPIO.setmode(GPIO.BCM)
for pin in pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    timers[pin] = None
    locks[pin] = Lock()
print "[GPIO Daemon] Setup of Autobed GPIO configuration complete."

# Callback for timer to set pin low after a delay
def pin_off(pin, cnum):
    with locks[pin]:
        GPIO.output(pin, GPIO.LOW)
        timers[pin] = None
    print "[GPIO Daemon] CB %s setting Pin %d LOW" % (cnum, pin)

# Set up socket listener, loop to receive and process connections with instructions from CGI-BIN script
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)
print "[GPIO Daemon] Ready, waiting for input."
count = 0
while True:
    conn, addr = sock.accept()  # get incoming connections from cgi-scripts
    cmd = conn.recv(32)  # Get data from connection.  Should have pin to raise;
    count += 1
    print "[GPIO Daemon] Received cmd # %s: %s" %(count, cmd)
    cmd = int(cmd.split('\n')[0])
    if cmd in pins:
        with locks[cmd]:  # lock access to the GPIO Pin and timer
            print timers[cmd]
            if timers[cmd] is not None:  # If timer present for pin, cancel it and leave pin (it's already high)
                timers[cmd].cancel()
    		print "[GPIO Daemon] Timer Canceled"
            else:
                GPIO.output(cmd, GPIO.HIGH)  # If no timer for pin, set pin high
    		print "[GPIO Daemon] Pin %d HIGH" % cmd 
            timers[cmd] = Timer(0.35, pin_off, [cmd, count])
	    print "[GPIO Daemon] New Timer for %s: %s" % (cmd, timers[cmd])
            timers[cmd].start()  # Create and Start new timer
        conn.send("success\r\n")
    else:
        print "[Autobed GPIO Daemon] Received unknown command: Pin %s" % cmd
        conn.send("failed\r\n")
