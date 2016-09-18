#!/bin/bash
cd `dirname $0` && \
sudo openvpn ./server.conf
