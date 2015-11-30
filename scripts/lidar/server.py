# server.py

import sys
import socket
import select
import traceback
import logging
from collections import deque
from array import array
from time import sleep

from dill import loads, dumps
import numpy as np


Instance = type(object())


class StreamHandler(object):

    def __init__(self, generator=None):
        pass

def typeData(data):
    if isinstance(data, np.ndarray):
        dataStr = str(data.shape)
    elif isinstance(data, str):
        dataStr = "length %d" % len(data)
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

    def __init__(self, HOST='', PORT=9009, exitTimeCallback=None):

        if exitTimeCallback is None:
            self.exitTimeCallback = lambda : False
        else:
            self.exitTimeCallback = exitTimeCallback

        self.SOCKET_LIST = []
        self.RECV_BUFFER = 2 ** 20

        self.messages = deque()
        self.lastMessage = None

        server_socket = self.server_socket = socket.socket(socket.AF_INET,
                                                           socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)

        # add server socket object to the list of readable connections
        self.SOCKET_LIST.append(server_socket)

        logging.info("Server started on port %s." % PORT)

    def getChar(self, nchars=1):
        logging.debug("Getting %d char(s)." % nchars)
        if nchars == 1:
            return self.getOneChar()
        else:
            chars = array('c', ['a']*nchars)
            for i in range(nchars):
                chars[i] = self.getOneChar()
            return chars

    def doWeHaveMessages(self):
        return len(self.messages) > 0

    def getOneChar(self):
        logging.debug("    Getting one char.")
        if self.lastMessage is None:
            haveMessages = self.doWeHaveMessages()
            niter = 0
            maxiter = 100
            while not haveMessages:  # Block until we have messages.
                if niter > maxiter:
                    # If we wait too long, put a dummy message on the deque.
                    self.messages.append([None])
                    break
                else:
                    niter += 1
                haveMessages = self.doWeHaveMessages()
                logging.debug("Blocking (iter %d) until messages are available." % niter)
                if self.getNumClients() > 0:
                    sleep(.01)
                else:
                    sleep(10)
            self.lastMessage = deque(self.messages.popleft())

        if len(self.lastMessage) == 0:
            self.lastMessage = None
            charGot = self.getOneChar()  # This shouldn't recurse more than once.
            logging.debug("    `->Returning char %s via a recursion." % hex(ord(charGot)))
            return charGot
        else:
            charGot = self.lastMessage.popleft()
            logging.debug("    `->Returning char %s." % hex(ord(charGot)))
            return charGot


    def getNumClients(self):
        return len(self.SOCKET_LIST) - 1

    def serve(self):
        while True:
            sleep(.00001)
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
                                self.messages.append(data)
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
            data = str(data)
        except (IndexError,) as e:
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

