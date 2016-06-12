#!/bin/bash
# Configuration is set in the file bootParams.
set -e
export nargs=$#
function retAbrt {
    if [[ "$nargs" -ge 3 ]]
    then
        echo Non-interactive mode. Continuing.
    else
        echo "Return to continue; Ctrl+C to abort."; read
    fi
}


bp=$1
user=$2
server=sigurd.tomsb.net
serverUser=tsbertalan
port=19898

echo Continuing with these parameters:
    echo Base dir: $bp
    echo SBC user: $user
    echo SSH target: $serverUser@$server:$port
echo Proceed?
retAbrt

## Generate SSH key files and install.
ssh-keygen -t rsa -N "" -f /tmp/id_rsa
cat /tmp/id_rsa.pub
echo "Installing public key in $HOME/.authorized_keys."
retAbrt
mkdir -p $HOME/.ssh
cat /tmp/id_rsa.pub >> $HOME/.ssh/authorized_keys
mkdir -p $bp/home/$user/.ssh
mv /tmp/id_rsa $bp/home/$user/.ssh/
mv /tmp/id_rsa.pub $bp/home/$user/.ssh/
cat ~/.ssh/id_rsa.pub >> $bp/home/$user/.ssh/authorized_keys



## Make ping.sh script to keep connection alive (I hope).
echo "Make ping script."
mkdir -p $bp
cat << EOF > $bp/etc/cron.hourly/pingServer.sh
#!/bin/bash
host=\`hostname\`
date=\`date\`
cmd="echo \$host : \$date >> ~/\${host}.status" 
echo "Running command"
echo "    \\$ \$cmd"
echo "on target \$1 from host \$host."
ssh "$serverUser@$server" "\$cmd"
EOF
chmod +x $bp/etc/cron.hourly/pingServer.sh



## Make boot install script, to be run exactly once.
desc="Install some things on first boot."
echo "Make script for: $desc"
cat << EOF >> $bp/etc/init.d/bootInstall
#!/bin/bash
### BEGIN INIT INFO
# Provides:          bootInstall
# Required-Start:
# Required-Stop:
# Default-Start: 2 3 4 5
# Default-Stop:
# Short-Description: $desc
# Description: $desc
### END INIT INFO
set -e
if [ -f /etc/bootInstall_semaphore ]
then
    echo "bootInstall has run before. Not running on this boot."
else
    apt-get clean
    apt-get update
    apt-get clean
    apt-get install -y --force-yes screen vim git
    chmod 700 /home/$user/.ssh
    chown -R $user:$user /home/$user/.ssh
    chmod 700 /home/$user/.ssh/
    chmod 600 /home/$user/.ssh/id_rsa*
    touch /var/spool/cron/crontabs/$user
    chown $user:$user /var/spool/cron/crontabs/$user
    chmod 600 /var/spool/cron/crontabs/$user
    chown $user:crontab /var/spool/cron/crontabs/$user
    # Console boot:
    [ -e /etc/init.d/lightdm ] && update-rc.d lightdm disable 2
        disable_boot_to_scratch
    # If the script has gotten to this point, we've succeeded. Touch a semaphore.
    touch /etc/bootInstall_semaphore
    echo "bootInstall appears to have succeeded. Rebooting..."
    reboot
fi
EOF
chmod +x $bp/etc/init.d/bootInstall
# Run at boot. 
cd $bp/etc/rc2.d && \
    ln -s ../init.d/bootInstall ./S99bootInstall



## Make script to start screen tunnel at boot.
desc="Start screen tunnel at boot."
echo "Make script for: $desc"
reverseCmd="ssh -o \"StrictHostKeyChecking no\" -tR $port:localhost:22 $serverUser@$server watch date"
cat << EOF >> $bp/etc/init.d/screenTunnel
#!/bin/bash
### BEGIN INIT INFO
# Provides:          screenTunnel
# Required-Start:
# Required-Stop:
# Default-Start: 2 3 4 5
# Default-Stop:
# Short-Description: $desc
# Description: $desc
### END INIT INFO
# Wait for network connection...try to ping $server for about a minute.
for i in {1..60}
do
    ping -c1 $server &> /dev/null && echo "Can see $server on network. Continuing...." && break
    echo "Can't see $server on network. Waiting..."
    sleep 1
done
su $user --command='/usr/bin/screen -dmS tunnel-screen $reverseCmd'
EOF
chmod +x $bp/etc/init.d/screenTunnel
# Run at boot.
cd $bp/etc/rc2.d && \
    ln -s ../init.d/screenTunnel ./S99screenTunnel



## Make WIFI defaults configuration file.
echo "Make WIFI defaults file."
wpadir=$bp/etc/wpa_supplicant/
mkdir -p $wpadir
cat << EOF >> $wpadir/wpa_supplicant.conf
network={
    ssid="puvisitor"
    priority=5
    key_mgmt=NONE
}
EOF
echo "" >> $wpaddr/wpa_supplicant.conf
cat extraNetworks.conf >> $wpaddr/wpa_supplicant.conf



## Set Keyboard layout and locale to US
echo "Set keyboard layout to US."
mkdir -p $bp/etc/default/
cat << EOF > $bp/etc/default/keyboard
XKBMODEL="pc104"
XKBLAYOUT="us"
XKBVARIANT=""
XKBOPTIONS=""

BACKSPACE="guess"
EOF
cat << EOF > $bp/etc/default/locale
LANG=en_US.UTF-8
EOF



echo "Done with script $0."
