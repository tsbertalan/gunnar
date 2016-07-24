# GUNNAR IS A ROBOT

## To make the SD card:

1. Download and extract Raspbian Jessie image (Find `2016-05-27-raspbian-jessie.img`
   somewhere on the internet, MD5 `78fdd2db5386e13b39c8d87bff1e7fe0`).
2. Mount image, alter it  to contain our backdoor, and unmount (script `mountAndAlterSD.sh`),
3. Write image (make targets `flash` and `verify`).
4. Insert SD into Raspberri Pi. Boot and wait for the script to install things and self-reboot.
5. If you can't SSH in after five minutes, hard reboot it.
5. If you're able to SSH in, run installROSrpi.sh to download and install ROS and friends.
