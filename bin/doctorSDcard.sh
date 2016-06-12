#!/bin/bash
set -e
function getAns {
    zenity --entry --text="$1" --entry-text="$2"
    }
bp=$( getAns "Path to card (containing etc/, bin/, etc.):" "/media/user/ADF23SDF234" )
user=$( getAns "User for SBC:" "pi" )

## Make ping.sh script to keep connection alive (I hope).
mkdir -p $bp
cat << EOF > $bp/bin/ping.sh
#!/bin/bash
host=`hostname`
date=`date`
cmd="echo \$host : \$date >> ~/\${host}.status" 
echo "Running command"
echo "    \\$ \$cmd"
echo "on target \$1 from host \$host."
ssh "\$1" "\$cmd"
EOF

## Make $user crontab to start screen tunnel at boot.
cat << EOF >> $bp/var/spool/cron/crontabs/$user
# m h  dom mon dow   command
@reboot  (. ~/.profile; /usr/bin/screen -dmS tunnel-screen $reverseCmd )
 *    * *  *   *     $HOME/bin/ping.sh $server > $HOME/Desktop/ping.sh.out 2>&1
EOF

## Make WIFI defaults configuration file.
cat << EOF >> $bp/etc/wpa_supplicant/wpa_supplicant.conf
network={
    ssid="puvisitor"
    priority=5
    key_mgmt=NONE
}
EOF

# mount:  sudo mount -o loop,offset=70254592 from-sd-card.img /mnt/img
