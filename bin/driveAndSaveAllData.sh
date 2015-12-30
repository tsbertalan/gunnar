#!/bin/bash
setPath="source $HOME/sketchbook/gunnar/bin/setPath.sh"
$setPath
HOST=sigurd.tomsb.net

echo "Starting LIDAR server on $HOST."
screen -dmS lidarServerSigurd ssh $HOST "$HOME/sketchbook/gunnar/bin/startLidarServer"
screen -list
sleep 8

echo "Starting LIDAR client on "`hostname`"."
screen -dmS lidarClientGunnar python $HOME/sketchbook/gunnar/bin/lidarLoggingClient.py $HOST
screen -list
sleep 8

echo "Starting controller on "`hostname`"."
screen -S controller python $HOME/sketchbook/gunnar/bin/driveAndSaveOdometry.py

