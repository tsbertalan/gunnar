#!/bin/bash
#
# This script is to be run on the robot at boot time.
#
source /home/tsbertalan/.profile
scrCmd="/usr/bin/screen -dmS"

# Start an SSH tunnel to sigurd, so we can SSH back into the robot.
cmd="$scrCmd tunnel-screen $HOME/bin/reverseTunnelSigurdStartup.sh"
echo \$ $cmd
$cmd 2>$HOME/Desktop/tunnelError.out

# Upload images from the cameras at regular intervals.
#sleep 1
#cmd="$scrCmd upload $HOME/sketchbook/gunnar/cam/uploadLastImageDir.py"
#echo \$ $cmd
#$cmd 2>$HOME/Desktop/uploadError.out

