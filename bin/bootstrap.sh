#!/usr/bin/env bash

# Some commands need to be run *not* as root. Run them as 
# userdo "COMMAND ARGS"
# (quoted and escaped, unfortunately)
function userdo { 
    sudo su vagrant --login --shell=/bin/bash --command "source ~/.profile ; $1"
}

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
userdo "echo \"source /opt/ros/kinetic/setup.bash\" >> ~/.profile"
userdo "rosdep update"

## Environment setup
userdo "source ~/.profile"




#### ARDUINO BUILDER ####
aptinst golang-go mercurial vim
userdo "git clone https://github.com/arduino/arduino-builder.git ~/arduinoBuilder"
userdo "mkdir -p ~/gocode"
userdo "mkdir -p ~/bin"
userdo "echo export GOPATH=\$HOME/arduinoBuilder >> ~/.profile"
userdo "go get github.com/go-errors/errors"
userdo "go get github.com/stretchr/testify"
userdo "go get github.com/jstemmer/go-junit-report"
userdo "go build arduino.cc/arduino-builder; mv arduino-builder ~/bin"
