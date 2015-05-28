#!/usr/bin/env python

import cgi
from arduino_driver import AutobedDriver # use for version with arduino intermediary
#from gpio_driver import AutobedDriver # use for version directly connected to GPIO of Raspberry Pi

#import cgitb
#cgitb.enable()

def returnMsg(msg):
    print "Content-type:text/plain\r\n\r\n"
    print msg

if __name__ == '__main__':
    form = cgi.FieldStorage()
    cmd = form['cmd'].value

    if cmd not in cmdMap:
        returnMsg("Error: Unknown Command \""+cmd+"\"")
    else:
        ctrl_char = cmdMap[cmd];
        try:
            driver = AutobedDriver()
            driver.send_command(ctrl_char)
            cmd_res = "Successful"
        except Exception as e:
            cmd_res = "Failed"

        if cmd == "headUP":
            cmd_txt = "Raise Head"
        elif cmd == "headDN":
            cmd_txt = "Lower Head"
        elif cmd == "bedUP":
            cmd_txt = "Raise Bed"
        elif cmd == "bedDN":
            cmd_txt = "Lower Bed"
        elif cmd == "legsUP":
            cmd_txt = "Raise Legs"
        elif cmd == "legsDN":
            cmd_txt = "Lower Legs"

        driver.close() # Clean up driver device/socket
        returncmd_txt(' '.join([cmd_txt, cmd_res]))
