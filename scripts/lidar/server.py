# server.py

import sys
import socket
import select
import traceback
import logging
from collections import deque

from dill import loads, dumps
import numpy as np


Instance = type(object())


class Handler(object):
    pass


class QueueHandler(Handler):
    def __init__(self):
        self.data = deque()

    def enquque(self, item):
        logging.info("Enququed %s." % typeData(item))
        self.data.append(item)

    def dequque(self):
        if len(self.data) > 0:
            item = self.data.popleft()
            logging.info("Deququed %s. %d left." % (typeData(item), len(self.data)))
            return item
        return False

class DummyHandler(Handler):

    def enquque(self, item):
        pass

    def dequque(self):
        pass



def typeData(data):
    if isinstance(data, np.ndarray):
        dataStr = str(data.shape)
    else:
        dataStr = str(data).strip()
    return "%s: '%s'" % (type(data), dataStr)


class Server(object):

    def removeSocket(self, sock):
        if sock in self.SOCKET_LIST:
            logging.info("Removing socket %s." % sock)
            self.SOCKET_LIST.remove(sock)
        else:
            logging.error("Socket %s not in socket list." % sock)

    def __init__(self, handler=None, HOST='', PORT=9009, exitTimeCallback=None):
        if handler is None:
            handler = DummyHandler()

        if exitTimeCallback is None:
            self.exitTimeCallback = lambda : False
        else:
            self.exitTimeCallback = exitTimeCallback

        self.handler = handler
        self.SOCKET_LIST = []
        self.RECV_BUFFER = 2 ** 16

        server_socket = self.server_socket = socket.socket(socket.AF_INET,
                                                           socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)

        # add server socket object to the list of readable connections
        self.SOCKET_LIST.append(server_socket)

        logging.info("Server started on port %s." % PORT)


    def serve(self):
        while True:
            if self.exitTimeCallback():
                break
            # get the list sockets which are ready to be read through select
            # 4th arg, time_out  = 0 : poll and never block
            ready_to_read, unused_ready_to_write, unused_in_error = select.select(self.SOCKET_LIST, [], [], 0)

            for sock in ready_to_read:
                # a new connection request recieved
                if sock == self.server_socket:
                    sockfd, addr = self.server_socket.accept()
                    if len(self.SOCKET_LIST) == 2:
                        logging.warn("Not accepting new connection request from [%s:%s]." % addr)
                    else:
                        self.SOCKET_LIST.append(sockfd)
                        logging.info("Client (%s, %s) connected." % addr)

                # a message from a client, not a new connection
                else:
                    # process data recieved from client,
                    try:
                        # receiving data from the socket.
                        try:
                            success, data = self.recv(sock, self.RECV_BUFFER)
                            if success:
                                # there is something in the socket
                                if isinstance(data, np.ndarray):
                                    self.handler.enquque(data)
#                             else:
#                                 logging.warn("False data: %s." % typeData(data))
                        except EOFError:
                            pass

                    except Exception:
                        logging.error(traceback.format_exc())
                        continue


    def recv(self, sock, length=2 ** 13, unpickle=True):
        data = sock.recv(length)
        try:
            data = loads(data)
        except (KeyError, IndexError) as e:
            import warnings
            warnings.warn(str(e))
            return False, None
        sys.stdout.flush()
        #logging.info("Got data: %s" % typeData(data))
        sys.stdout.flush()
        return True, data

    def stop(self):
        self.server_socket.close()

if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    s = Server()
    sys.exit(s.serve())

