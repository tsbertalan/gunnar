#!/usr/bin/env bash



# The httpredir.debian.org "magic" mirror which debian64 uses is buggy.
# We might need to hit it multiple times. Fortunately, a successful
# apt-get install is idempotent.
function aptinst {
    sudo apt-get update
    for i in 1 2 3 4
    do
        sudo apt-get install -y $@
        sleep 4
    done
}




#### ROBOT OPERATING SYSTEM ####
# instructions from http://wiki.ros.org/kinetic/Installation/Debian

## Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

## Installation
aptinst ros-kinetic-desktop-full python-rosinstall

## Initialize rosdep
sudo rosdep init
echo "source /opt/ros/kinetic/setup.bash" >> ~/.profile
rosdep update

## Environment setup
source ~/.profile




#### ARDUINO BUILDER ####
# aptinst golang-go mercurial vim arduino arduino-core
# git clone https://github.com/arduino/arduino-builder.git ~/arduinoBuilder
# mkdir -p ~/gocode
# mkdir -p ~/bin
# echo "export GOPATH=\$HOME/arduinoBuilder" >> ~/.profile
# source ~/.profile
# go get github.com/go-errors/errors
# go get github.com/stretchr/testify
# go get github.com/jstemmer/go-junit-report
# go build arduino.cc/arduino-builder; mv arduino-builder ~/bin

# Install unmentioned dependencies.
# aptinst libpococrypto9-dbg libpocodata9-dbg libpocomysql9-dbg \
#     libpoconet9-dbg libpoconetssl9-dbg libpocoodbc9-dbg libpocosqlite9-dbg \
#     libpocozip9-dbg
# # Fix the broken libgif-dev install.
# # https://bugs.launchpad.net/ubuntu/+source/graphviz/+bug/1398037
# sudo ln -s `locate libgif.a | tail -n 1` /usr/lib/libgif.a
# sudo ln -s `locate libgif.a | tail -n 1` /usr/lib/libgif.la
# sudo ln -s `locate libgif.so.4.1.6 | tail -n 1` /usr/lib/libgif.so.4.1.6




#### ARDUINO-MK ####
# arduino-builder seems to be plagued with bugs at the moment. We keep to the old ways.
sudo apt-get install -y arduino arduino-core arduino-mk
