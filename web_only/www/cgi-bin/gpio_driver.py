#!/usr/bin/env python

import socket

class AutobedDriver(object):
    cmdMap = {'headUP': '17\n',
              'headDN': '18\n',
              'bedUP':  '27\n',
              'bedDN':  '22\n',
              'legsUP': '23\n',
              'legsDN': '24\n'}

    def __init__(self, host="127.0.0.1", port=828):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send_command(self, cmd):
        if cmd in self.cmdMap:
            self.socket.connect((self.host, self.port))
            self.socket.sendall(self.cmdMap[cmd])
            resp = self.socket.recv(128)
            self.socket.close()
            if "success" in resp:
                return "Succeeded"
            else:
                return "Failed"
        else:
            raise RuntimeException("[Autobed Driver] Received Unknown Command %s" %cmd)

    def close(self):
        self.socket.close()
