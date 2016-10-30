#!/bin/bash
cd `dirname $0` && \
sudo openvpn --mute-replay-warnings --config ./server.conf
