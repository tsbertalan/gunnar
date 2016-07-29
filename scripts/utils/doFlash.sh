#!/bin/bash
IMGPATH=$1
SDX=$2

set -e

echo
echo "Device chosen for flashing:"
python scripts/utils/deviceInfo.py $SDX
echo
echo "MAKE SURE THE CORRECT DEVICE IS CHOSEN!!!"
echo "Press return to continue or Ctrl+C to abort."; read
sudo parted --script $SDX mkpart primary ext4 5G || true
sudo dcfldd bs=4M if=$IMGPATH of=$SDX
sync
