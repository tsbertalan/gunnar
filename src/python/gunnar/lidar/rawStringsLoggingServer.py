# Log data from Neato LIDAR via socket server

import logging
from threading import Thread
from time import sleep

from gunnar.io.network import Server
from gunnar.io.disk import PyTableSavingHandler as SavingHandler
from parseLidar import LidarParser


class Watcher():

    def __init__(self):
        self.exitNow = False

    def watch(self):
        msg = "Type 'exit' and push enter to quit:\n"
        while True:
            try:
                sleep(0.00001)  # do not hog the processor power
                inputText = raw_input(msg).lower()
                logging.debug("Input text was %s." % inputText)
                self.exitNow = 'exit' in inputText
                if self.exitNow:
                    logging.debug("Should be exiting now.")
                    sleep(1.0)
                    break
            except KeyboardInterrupt:
                self.exitNow = True
                break


    def exitNowCallback(self):
        return self.exitNow


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)

    watcher = Watcher()
    server = Server(exitTimeCallback=watcher.exitNowCallback)
    parser = LidarParser(server, exitTimeCallback=watcher.exitNowCallback)
    handler = SavingHandler('data')
    def watchForNewParsedArrays():
        while True:
            sleep(0.00001)  # do not hog the processor power
            if watcher.exitNowCallback():
                break
            if len(parser) > 0:
                handler.enquque(parser.pop())


    threads = []
    for target in server.serve, parser.parse, watchForNewParsedArrays:
        threads.append(Thread(target=target))
        threads[-1].start()
        sleep(.1)

    watcher.watch()
