# From this guide:
# http://wiki.ros.org/indigo/Installation/UbuntuARM
# claims to be for Ubuntu Trusty, but also appears to work for Raspbian Jessie.

# Attempt to exit script on errors.
set -e

# Make sure debconf knows we won't be interacting with it.
# http://serverfault.com/questions/500764
export DEBIAN_FRONTEND=noninteractive

# Set local environment variables for unprivelidged home and username. 
user=pi
userhome=/home/$user/
gitdir=$userhome/catkin_ws/src/gunnar
su="sudo -i -u $user"

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
aptinst ros-indigo-geometry python-rosdep ros-indigo-xv-11-laser-driver \
ros-indigo-teleop-twist-joy ros-indigo-yocs-velocity-smoother

# For viso2 (failed)
#aptinst ros-indigo-image-transport ros-indigo-cv-bridge ros-indigo-image-geometry ros-indigo-pcl-ros

# For SVO
aptinst ros-indigo-rqt-gui ros-indigo-rqt-gui-py ros-indigo-cmake-modules



# Download SVO code and dependencies.
export CATKIN_WS=$userhome/catkin_ws/src/
$su mkdir -p $CATKKIN_WS && cd $CATKKIN_WS
$su git clone https://github.com/uzh-rpg/rpg_vikit.git
$su git clone https://github.com/uzh-rpg/rpg_svo.git
$su git clone https://github.com/ros-perception/vision_opencv.git && cd vision_opencv && git checkout indigo
$su cd $CATKKIN_WS
# patch for ARM compilation
export ARM_ARCHITECTURE=armv7l
export IS_ARM=true
for packagename in rpg_vikit rpg_svo
do
    $su cd $CATKIN_WS/$packagename && git apply $gitdir/scripts/utils/$packagename.patch
done
 
# This might not be necessary with catkin_make called later, but this is what worked for me.
$su rosdep install vision_opencv

# Some non-ros packages (see https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-ROS):
cmakeWorkspace=$userhome/cmakeWorkspace/
$su mkdir -p $cmakeWorkspace
cd $cmakeWorkspace
$su git clone https://github.com/strasdat/Sophus.git
cd Sophus
$su git checkout a621ff
$su mkdir build
cd build
$su cmake ..
$su make

cd $cmakeWorkspace
$su git clone https://github.com/uzh-rpg/fast.git
$su cd fast
$su mkdir build
cd biuld
$su cmake ..
$su make

# (skipping g2o)



	
# Install some non-ROS packages.
aptinst python-pip arduino ipython git python-matplotlib network-manager

# Install xbox controller drivers.
sudo apt-get install --yes --force-yes --install-recommends jstest* joystick xboxdrv
# We also need to ensure that xpad is not getting loaded:
echo "blacklist xpad" | sudo tee -a /etc/modprobe.d/blacklist.conf
sudo rmmod xpad  # unload module if already loaded
	
# Install some Python packages.
sudo pip install rosdep rosinstall_generator wstool rosinstall
	
# Initialize ROS.
sudo rosdep init
rosdep update

# Set development desktop as git remote.
$su bash -l -c "cd $gitdir && git remote remove origin"
$su bash -l -c "cd $gitdir && git remote add origin tsbertalan@sigurd.tomsb.net:~/workspace/gunnar"
$su bash -l -c "cd $gitdir && git fetch"
$su bash -l -c "cd $gitdir && git branch -u origin/master master"
    
# Temporarily add swap space for comilation.
sudo patch /etc/dphys-swapfile -i $gitdir/scripts/utils/swapup.patch
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start

# Try to build the workspace.
$su bash -l -c "cd /home/$user/catkin_ws && catkin_make"

# Set swap allowance back to lower amount.
sudo patch /etc/dphys-swapfile -i $gitdir/scripts/utils/swapdown.patch
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start

# If the script has gotten to this point, we've succeeded. Touch a semaphore.
sudo touch /etc/bootInstall_semaphore
sudo reboot

