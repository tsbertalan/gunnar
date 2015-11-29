# client.py

import sys
import socket
from time import sleep
import select
from time import time
from collections import deque
import traceback

from dill import dumps, loads
import numpy as np


class Client(object):
    def __init__(self, host, port, timeout=2):
        self.nsent = 0
        self.host = host
        self.port = port

        self.s = s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)

        # connect to remote host
        try :
            s.connect((host, port))
        except Exception:
            traceback.print_exc()
            print 'Unable to connect'
            sys.exit()

        print 'Connected to remote host. You can start sending messages'
        sys.stdout.flush()

        self.messages = deque()

    def send(self, obj):
        data = dumps(obj)
        self.nsent += 1
        #print "sending message %d: object of type %s, len %d" % (self.nsent, type(obj), len(data))
        self.s.send(data)


class Message(object):
    def __init__(source, content):
        self.source = source
        self.content = content


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "USAGE:", sys.argv[0], "HOST", "PORT"
        from sys import exit; exit()

    client = Client(sys.argv[1], int(sys.argv[2]))
    sleep(1.0)
    client.send("hello")

