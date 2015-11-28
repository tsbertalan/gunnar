# server.py
 
import sys
import socket
import select
import traceback
import logging

from dill import loads, dumps
Instance = type(object())

from collections import deque
import numpy as np

class Handler(object):
    def __init__(self):
        self.data = deque()
    def enque(self, item):
        self.data.append(item)
    def work(self):
        if len(self.data) > 0:
            item = self.data.popleft()
            logging.info("De-que'd %s : '%s'" % (type(item), str(item).strip()))

def typeData(data):
    return "%s: '%s'" % (type(data), str(data).strip())

def recv(sock, length=4096, unpickle=True):
    data = sock.recv(length)
    if isinstance(data, bool) and not data:
        return False, None
    if unpickle:
        data = loads(data)
    sys.stdout.flush()
    msg = ">>>>"
    msg += "\n    RECV():"
    msg += "\n    Got data: %s" % typeData(data)
    if isinstance(data, Instance) and not (
        isinstance(data, str) or isinstance(data, np.ndarray)
        ):
        msg += "\n    object fields:"
        for key in dir(data):
            val = getattr(data, key)
            msg += "\n        \"%s\":: %s" % (key, typeData(val))
    if hasattr(data, "doubleTime"):
        msg += "\ndata times:", data.time, data.doubleTime()
    msg += "\n<<<<"
    logging.info(msg)
    sys.stdout.flush()
    return True, data


SOCKET_LIST = []
def server(HOST='', PORT=9009):
    RECV_BUFFER = 4096 
    def removeSocket(sock):
        if sock in SOCKET_LIST:
            logging.info("Removing socket %s." % sock)
            SOCKET_LIST.remove(sock)
        else:
            logging.error("Socket %s not in socket list." % sock)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(10)
 
    # add server socket object to the list of readable connections
    SOCKET_LIST.append(server_socket)
 
    logging.info("Server started on port " + str(PORT))

    handler = Handler()
 
    while 1:
        handler.work()

        # get the list sockets which are ready to be read through select
        # 4th arg, time_out  = 0 : poll and never block
        ready_to_read,ready_to_write,in_error = select.select(SOCKET_LIST,[],[],0)
      
        for sock in ready_to_read:
            # a new connection request recieved
            if sock == server_socket: 
                sockfd, addr = server_socket.accept()
                if len(SOCKET_LIST) == 2:
                    logging.warn("Not accepting new connection request from [%s:%s]." % addr)
                else:
                    SOCKET_LIST.append(sockfd)
                    logging.info("Client (%s, %s) connected" % addr)
                 
                    broadcast(server_socket, sockfd, "[%s:%s] entered the server." % addr)
             
            # a message from a client, not a new connection
            else:
                # process data recieved from client, 
                try:
                    # receiving data from the socket.
                    success, data = recv(sock, RECV_BUFFER)
                    if success:
                        # there is something in the socket
                        if isinstance(data, np.ndarray):
                            handler.enque(data)
                    else:
                        # remove the socket that's broken    
                        logging.warn("False data: %s" % typeDat(data))
                        removeSocket(sock)

                        # at this stage, no data means probably the connection has been broken
                        logging.warn("Client (%s, %s) is offline" % addr)

                # exception 
                except EOFError:
                    logging.warn(traceback.format_exc())
                    logging.warn("Lost connection (client killed?).")
                    removeSocket(sock)
                    continue
                except Exception:
                    logging.error(traceback.format_exc())
                    continue

    server_socket.close()
    
# broadcast messages to all connected clients
def broadcast (server_socket, sock, message, pickle=True):
    logging.info("sending data %s" % typeData(message))
    if pickle:
        message = dumps(message)
    for socket in SOCKET_LIST:
        # send the message only to peer
        if socket != server_socket and socket != sock :
            try :
                socket.send(message)
            except :
                # broken socket connection
                socket.close()
                # broken socket, remove it
                if socket in SOCKET_LIST:
                    SOCKET_LIST.remove(socket)
 
if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    sys.exit(server()) 

