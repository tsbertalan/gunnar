#!/bin/bash
python recordSerial.py &
while [ 1 ]; do python cam/cam.py dodate continuous; done