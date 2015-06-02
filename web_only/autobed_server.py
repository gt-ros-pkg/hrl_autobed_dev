#!/usr/bin/env python

import RPi.GPIO as GPIO

from tornado.web import Application, url
from tornado.websocket import WebSocketHandler
from tornado.ioloop import IOLoop

import signal
from sys import exit

PORT = 828
cmdMap = {'headUP': 23,
          'headDN': 27,
          'bedUP': 25,
          'bedDN': 24,
          'legsUP': 4,
          'legsDN': 17}
timers = {}

class AutobedWebSocketHandler(WebSocketHandler):
    def check_origin(self, origin):
        return True
    
    def open(self):
        print "New WebSocket client connected from %s." %(self.request.remote_ip)
        self.set_nodelay(True)

    def on_message(self, message):
#        self.write_message("Received: %s" % message)
        if message not in cmdMap:
            print "Received unknown command: %s" %message
            self.write_message("Received unknown command: %s" %message)
        else:
            pin = cmdMap[message]
            if timers[pin] is None:
                GPIO.output(pin, GPIO.HIGH)
            else:
                IOLoop.current().remove_timeout(timers[pin])
            #client sends msgs at 75ms intervals, this will stay up if one is missed entirely
            timers[pin] = IOLoop.current().call_later(0.155, self.reset_pin, pin) 

    def on_close(self):
        print "Connection from %s closed." %(self.request.remote_ip)

    def reset_pin(self, pin):
        GPIO.output(pin, GPIO.LOW)
        timers[pin] = None
        print "Reset %d" %pin

def GPIO_setup():
    GPIO.setmode(GPIO.BCM)
    for pin in cmdMap.values():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        timers[pin] = None
    print "AutoBed GPIO configuration complete."

def GPIO_cleanup(signalnum, frame):
    for pin in cmdMap.values():
        GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()
    print "Autobed GPIO cleanup complete. Exiting."
    exit(0)

if __name__=='__main__':
    signal.signal(signal.SIGINT, GPIO_cleanup)
    signal.signal(signal.SIGTERM, GPIO_cleanup)
    GPIO_setup()

    application = Application([url(r"/", AutobedWebSocketHandler)])
    application.listen(PORT)
    print "Autobed Server Running..."
    IOLoop.current().start()
