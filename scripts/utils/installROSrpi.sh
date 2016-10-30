# From this guide:
# http://wiki.ros.org/indigo/Installation/UbuntuARM
# claims to be for Ubuntu Trusty, but also appears to work for Raspbian Jessie.

# Make sure debconf knows we won't be interacting with it.
# http://serverfault.com/questions/500764
export DEBIAN_FRONTEND=noninteractive


# Set locale stuff (optional?).
# Some of this seems to fail anyway.
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo locale-gen en_US.UTF-8
# probably optional-er:
#sudo dpkg-reconfigure locales
function aptinst {
	echo "Installing: $@"
    for i in 1 2 3 4
    do
        sudo apt-get install --yes --force-yes $@
        sleep 4
    done
}

# Upgrade existing software.
sudo aptitude upgrade --assume-yes
aptinst vim htop

# Add apt source/key.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

# Instal ROS Indigo base. 
aptinst ros-indigo-ros-base

# Install other ROS packages.
aptinst ros-indigo-geometry python-rosdep ros-indigo-xv-11-laser-driver
	
# Install some non-ROS packages.
aptinst python-pip arduino ipython git python-matplotlib network-manager
	
# Install some Python packages.
sudo pip install rosdep rosinstall_generator wstool rosinstall
	
# Initialize ROS.
sudo rosdep init
rosdep update

# Set development desktop as git remote.
user=pi
gitdir=/home/$user/catkin_ws/src/gunnar
su="sudo -i -u $user"
$su bash -l -c "cd $gitdir && git remote remove origin"
$su bash -l -c "cd $gitdir && git remote add origin tsbertalan@sigurd.tomsb.net:~/workspace/gunnar"
$su bash -l -c "cd $gitdir && git fetch"
$su bash -l -c "cd $gitdir && git branch -u origin/master master"
    
# Try to build the workspace.
$su bash -l -c "cd /home/$user/catkin_ws && catkin_make"

# If the script has gotten to this point, we've succeeded. Touch a semaphore.
sudo touch /etc/bootInstall_semaphore
sudo reboot

