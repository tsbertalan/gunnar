#!/bin/bash
source /home/tsbertalan/.profile
scrCmd="/usr/bin/screen -dmS"

cmd="$scrCmd tunnel-screen $HOME/bin/reverseTunnelSigurdStartup.sh"
echo \$ $cmd
$cmd 2>$HOME/Desktop/tunnelError.out
sleep 1
cmd="$scrCmd upload $HOME/sketchbook/gunnar/cam/uploadLastImageDir.py"
echo \$ $cmd
$cmd 2>$HOME/Desktop/uploadError.out

