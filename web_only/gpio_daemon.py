#!/usr/bin/env python

import socket
import RPi.GPIO as GPIO
from threading import Timer, Lock
import atexit

#socket parameters
HOST = '127.0.0.1'  # only listen to loopback interface (only connections from this machine)
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
            GPIO.write(pin, GPIO.LOW)  # Set pins low
            timers[pin].cancel()  # Stop timers
            timers[pin] = None
    GPIO.cleanup()

# Setup GPIO settings, make timer and locks for each pin, set each pin to OUTPUT, LOW (off).
GPIO.setmode(GPIO.BCM)
for pin in pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    timers[pin] = None
    locks[pin] = Lock()

# Callback for timer to set pin low after a delay
def pin_off(pin):
    with locks[pin]:
        GPIO.write(pin, GPIO.LOW)
        timers[pin] = None

# Set up socket listener, loop to receive and process connections with instructions from CGI-BIN script
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)
while True:
    conn, addr = sock.accept()  # get incoming connections from cgi-scripts
    cmd = conn.recv(32).split('\n')[0]  # Get data from connection.  Should have pin to raise;
    if cmd in pins:
        with locks[cmd]:  # lock access to the GPIO Pin and timer
            if timers[cmd] is not None:  # If timer present for pin, cancel it and leave pin (it's already high)
                timers[cmd].cancel()
            else:
                GPIO.write(cmd, GPIO.HIGH)  # If no timer for pin, set pin high
            timers[cmd] = Timer(0.35, pin_off, [cmd])  # Start new timer
        conn.send("success\r\n")
    else:
        print "[Autobed GPIO Daemon] Received unknown command: Pin %s" % cmd
        conn.send("failed\r\n")
