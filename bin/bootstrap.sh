#!/usr/bin/env bash
# Some commands need to be run *not* as root. Run them as 
# $userdo "COMMAND ARGS"
userdo="sudo su vagrant -c "

#### ROBOT OPERATING SYSTEM SETUP ####
# instructions from http://wiki.ros.org/kinetic/Installation/Debian

## Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

## Installation
sudo apt-get update
# The httpredir.debian.org "magic" mirror which debian64 uses is buggy.
# We might need to hit it multiple times. Fortunately, a successful
# apt-get install is idempotent.
for i in 1 2 3 4
do
    sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall
    sleep 4
done

## Initialize rosdep
sudo rosdep init
$userdo "echo \"source /opt/ros/kinetic/setup.bash\" >> ~/.bashrc"
$userdo "rosdep update"

## Environment setup
$userdo "source ~/.bashrc"

