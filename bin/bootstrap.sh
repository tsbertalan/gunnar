#!/usr/bin/env bash
# Attempt to exit on first error.
set -e


# The httpredir.debian.org "magic" mirror which debian64 uses is buggy.
# We might need to hit it multiple times. Fortunately, a successful
# apt-get install is idempotent.
sudo apt-get update
function aptinst {
    for i in 1 2 3 4
    do
        sudo apt-get install -y $@
        sleep 4
    done
}




#### ROBOT OPERATING SYSTEM ####
## Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get upgrade -y

## Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

#     #########
#     distDesc=$(lsb_release --description)
#     if [ $distDesc == "Description:    Raspbian GNU/Linux 8.0 (jessie)" ]
#     then
#     #########

# Instructions from http://wiki.ros.org/indigo/Installation/UbuntuARM,
# but mostly from 
# http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
# If the precompiled route (the first link) can be made to work, it would be
# preferable.

# Set your Locale
# Boost and some of the ROS tools require that the system locale be set. You can set it with:
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Install Bootstrap Dependencies
aptinst python-pip python-setuptools python-yaml python-distribute \
    python-docutils python-dateutil python-six libconsole-bridge-dev \
    liburdfdom-headers-dev liburdfdom-dev liblz4-dev
sudo pip install rosdep rosinstall_generator wstool rosinstall

# Initializing rosdep
sudo rosdep init
rosdep update

# Installation
# Now, we will download and build ROS Indigo.

# Create a catkin Workspace
# In order to build the core packages, you will need a catkin workspace.
# Create one now:
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

# Next we will want to fetch the core packages so we can build them. We will
# use wstool for this. Select the wstool command for the particular variant
# you want to install:

# ROS-Comm: (recommended) ROS package, build, and communication libraries.
# No GUI tools.
# rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
# wstool init src indigo-ros_comm-wet.rosinstall

# Desktop: ROS, rqt, rviz, and robot-generic libraries
rosinstall_generator desktop --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-desktop-wet.rosinstall

# Unavailable Dependencies
# Before you can build your catkin workspace, you need to make sure that you
# have all the required dependencies. We use the rosdep tool for this, however,
# a couple of dependencies are not available in the repositories. They must be
# manually built first.
# For jessie, we need collada-dom-dev.

# # The required packages can be built from source in a new directory:
# mkdir -p ~/ros_catkin_ws/external_src
# sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
# sudo apt-get update
# aptinst checkinstall cmake libboost-filesystem-dev libxml2-dev
# 
# # collada-dom-dev:
# cd ~/ros_catkin_ws/external_src
# wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
# tar -xzf collada-dom-2.4.0.tgz
# cd collada-dom-2.4.0
# 
# # You will also need to patch collada_urdf as described at
# # https://github.com/ros/robot_model/issues/12
# # and
# # https://groups.google.com/forum/#!msg/ros-sig-embedded/26XlDtZhyNs/OexZAx6BCBcJ
# # Make the package
# cmake .
# sudo checkinstall make install
# # When check-install asks for any changes, the name (2) needs to change from "collada-dom" to "collada-dom-dev" otherwise the rosdep install wont find it.

# Note: If you don't want to compile Collada but would like to install the
# desktop variant, use the following generator:
rosinstall_generator desktop --rosdistro indigo --deps --wet-only --exclude roslisp collada_parser collada_urdf --tar > indigo-desktop-wet.rosinstall

# Populate ./src.
wstool init src indigo-desktop-wet.rosinstall


# Resolving Dependencies with rosdep
# The remaining dependencies should be resolved by running rosdep:
cd ~/ros_catkin_ws && \
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:$(lsb_release -sc)


# For rviz, you will also have to apply this patch:
# http://answers.ros.org/question/52098/unable-to-compile-rviz-on-ubuntu-armhf/
patch -p0 < ~/sketchbook/gunnar/src/install/mesh_loader.patch


# Building the catkin Workspace
# Once you have completed downloading the packages and have resolved the 
# dependencies, you are ready to build the catkin packages.
# Note: This will install ROS in the equivalent file location to Ubuntu in /opt/ros/indigo however you can modify this as you wish.
# Invoke catkin_make_isolated:
cd ~/ros_catkin_ws && \
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j1


#     #########
#     else  # Description:    Raspbian GNU/Linux 8.0 (jessie)
#     #########
# 
# # instructions from http://wiki.ros.org/kinetic/Installation/Debian
# 
# ## Installation
# aptinst ros-indigo-desktop-full python-rosinstall
# 
# ## Initialize rosdep
# sudo rosdep init
# echo "source /opt/ros/kinetic/setup.bash" >> ~/.profile
# rosdep update
# 
# ## Environment setup
# source ~/.profile
# 
#     #########
#     end
#     #########





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



#### BUILD ####
cd ~/sketchbook/gunnar && make
