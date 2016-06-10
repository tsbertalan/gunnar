#!/usr/bin/env bash

#### ROBOT OPERATING SYSTEM SETUP ####
# instructions from http://wiki.ros.org/kinetic/Installation/Debian

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

# Installation
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall

# Initialize rosdep
sudo rosdep init
whoami
sudo su vagrant
whoami
rosdep update

# Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

