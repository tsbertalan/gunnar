## Mount a Raspberry Pi SD image and do stuff to it.
#
source `dirname $0`/mountSDcard.sh

echo Add things to partition mounted at $mntPoint.
sudo `dirname $0`/doctorSDcard.sh $mntPoint $user

echo Unmount $mntPoint.
sudo umount $mntPoint

