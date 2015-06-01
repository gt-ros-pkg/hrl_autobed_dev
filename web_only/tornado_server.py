#!/usr/bin/env python

from tornado.web import Application, url
from tornado.websocket import WebSocketHandler
from tornado.ioloop import IOLoop

import RPi.GPIO as GPIO

import atexit

PORT = 8028
cmdMap = {'headUP': 17,
          'headDN': 18,
          'bedUP': 27,
          'bedDN': 22,
          'legsUP': 23,
          'legsDN': 24}
timers = {}

class AutobedWebSocketHandler(WebSocketHandler):

    def check_origin(self, origin):
        return True
    
    def open(self):
        print "New Connection Opened from %s." %(self.request.remote_ip)
        self.set_nodelay(True)

    def on_message(self, message):
        self.write_message("Received: %s" % message)
        if message not in cmdMap:
            self.write_message("Received unknown command: %s" %message);
        else:
            pin = cmdMap[message]
            if timers[pin] is None:
                GPIO.output(pin, GPIO.HIGH)
            else:
                IOLoop.current().remove_timeout(timers[pin])
            timers[pin] = IOLoop.current().call_later(0.5, self.reset_pin, pin)

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

def GPIO_cleanup():
    for pin in cmdMap.values():
        GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()
    print "Autobed GPIO cleanup complete."

if __name__=='__main__':
    atexit.register(GPIO_cleanup)
    GPIO_setup()

    application = Application([url(r"/", AutobedWebSocketHandler)])
    application.listen(PORT)
    IOLoop.current().start()
