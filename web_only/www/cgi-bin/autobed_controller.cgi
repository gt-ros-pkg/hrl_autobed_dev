#!/usr/bin/env python

import cgi

import cgitb
cgitb.enable()

def returnMsg(msg):
    print "Content-type:text/plain\r\n\r\n"
    print msg

if __name__ == '__main__':
    form = cgi.FieldStorage()
    cmd = form['cmd'].value

    if cmd not in cmdMap:
        returnMsg("Error: Unknown Command \""+cmd+"\"")
    else:
        ctrlChar = cmdMap[cmd];

        if cmd == "headUP":
            msg = "Raise Head"
        elif cmd == "headDN":
            msg = "Lower Head"
        elif cmd == "bedUP":
            msg = "Raise Bed"
        elif cmd == "bedDN":
            msg = "Lower Bed"
        elif cmd == "legsUP":
            msg = "Raise Legs"
        elif cmd == "legsDN":
            msg = "Lower Legs"

        returnMsg(msg+" Successful")
