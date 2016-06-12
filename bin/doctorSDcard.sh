#!/bin/bash
# Configuration is set in the file bootParams.
set -e
function retAbrt {
    echo "Return to continue; Ctrl+C to abort."; read
    }
function par {
    `dirname $0`/bootParams $1
    }

echo Continuing with these parameters:
`dirname $0`/bootParams print
echo Proceed?
retAbrt

bp=$( par bp )
user=$( par user )
server=$( par server )
serverUser=$( par serverUser)
port=$( par port )



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



## Make ping.sh script to keep connection alive (I hope).
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
cat << EOF >> $bp/etc/init.d/bootInstall
#!/bin/bash
set -e
if [ -f /etc/bootInstall_semaphore ]
then
    echo "bootInstall has run before."
else
    apt-get install -y screen vim git
    chmod 700 /home/$user/.ssh
    chown $user:$user /home/$user/.ssh
    chmod 700 /home/$user/.ssh/
    chmod 600 /home/$user/.ssh/id_rsa*
    chown $user:$user /var/spool/cron/crontabs/$user
    chmod 600 /var/spool/cron/crontabs/$user
    chown $user:crontab /var/spool/cron/crontab/$user
    # If the script has gotten to this point, we've succeeded. Touch a semaphore.
    touch /etc/bootInstall_semaphore
fi
EOF
chmod +x $bp/etc/init.d/bootInstall
# Run at boot. 
cd $bp/etc/rc2.d && \
    ln -s ../init.d/bootInstall ./S99bootInstall



## Make script to start screen tunnel at boot.
reverseCmd="ssh -o \"StrictHostKeyChecking no\" -tR $port:localhost:22 $serverUser@$server watch date"
cat << EOF >> $bp/etc/init.d/screenTunnel
/usr/bin/screen -dmS tunnel-screen $reverseCmd
EOF
# Run at boot.
cd $bp/etc/rc2.d && \
    ln -s ../init.d/screenTunnel ./S99screenTunnel



## Make WIFI defaults configuration file.
mkdir -p $bp/etc/wpa_supplicant
cat << EOF >> $bp/etc/wpa_supplicant/wpa_supplicant.conf
network={
    ssid="puvisitor"
    priority=5
    key_mgmt=NONE
}
EOF

# mount:  sudo mount -o loop,offset=70254592 from-sd-card.img /mnt/img
