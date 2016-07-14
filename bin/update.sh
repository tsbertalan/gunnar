#!/bin/sh
# DEPRECATED: Update a pcduino image.
cd `dirname $0`
IMG=pcduino_ubuntu_20131126.img

IMG_SIZE=`du -s $IMG  | cut -f1`
BURN_TIME=`expr $IMG_SIZE / 1024 / 3 / 60`
echo -e "\twriting $IMG to nand flash\n"
echo -e "\tit will take about $BURN_TIME minutes to finish..."

time dd if=$IMG of=/dev/nandd bs=4M && sync
if [ $? -eq 0 ]; then
    echo "update finished"
    killall blink_led.sh
    /blink_led.sh 18 1000000 & 
    /blink_led.sh 19 1000000 &          
else                                        
    echo "write ubuntu to nand failed"
    killall blink_led.sh   
    /blink_led.sh 18 100000 &
    /blink_led.sh 19 100000 &
    exit 1
fi
