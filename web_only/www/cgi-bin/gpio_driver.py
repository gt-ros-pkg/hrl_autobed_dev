#!/usr/bin/env python

import socket

cmdMap = {'headUP': 17,
          'headDN': 18,
          'bedUP': 27,
          'bedDN': 22,
          'legsUP': 23,
          'legsDN': 24}

class AutobedDriver(object)
    def __init(self, host="127.0.0.1", port=828):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send_command(self, cmd):
        if cmd in cmdMap:
            self.socket.connect((self.host, self.port))
            self.socket.sendall(cmdMap[cmd])
            resp = self.socket.recv(256)
            self.socket.close()
            if "success" in resp:
                return "Succeeded"
            else:
                return "Failed"
        else:
            raise RuntimeException("[Autobed Driver] Received Unknown Command %s" %cmd)

    def close(self):
        self.socket.close()
