# GUNNAR IS A ROBOT

## To make the SD card:

1. Download and extract Raspbian Jessie image (make target `dlUbuntu`, maybe).
2. Mount image, alter it  to contain our backdoor, and unmount (script `mountAndAlterSD.sh`),
3. Write image (make targets `flash` and `verify`).
4. Insert SD into Raspberri Pi. Boot and wait for some stuff to install.
5. If you can't SSH in after five minutes, hard reboot it.
5. If you're able to SSH in, run buildROSrpi.sh to download and build ROS and friends.
