# From this guide:
# http://wiki.ros.org/indigo/Installation/UbuntuARM
# claims to be for Ubuntu Trusty, but also appears to work for Raspbian Jessie.

# Set locale stuff (optional?).
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo locale-gen en_US.UTF-8
# probably optional-er:
#sudo dpkg-reconfigure locales

# Add apt source/key.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

# Instal ROS Indigo base. 
sudo apt-get install ros-indigo-ros-base

# Install other ROS packages.
sudo apt-get install ros-indigo-geometry python-rosdep
	
# Install some non-ROS packages.
sudo apt-get install python-pip arduino ipython vim git
	
# Install some Python packages.
sudo pip install rosdep rosinstall_generator wstool rosinstall
	
	
# Initialize ROS.
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws
