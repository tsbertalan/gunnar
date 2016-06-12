#!/bin/bash
# run like:
# source <(curl -s https://raw.githubusercontent.com/tsbertalan/gunnar/master/bin/bootstrapSSH.sh)
echo Enter user@server for reverse SSH tunnel:
read server
echo "Got user@server: $server. Ctrl+c to abort and retry; return to continue."
read
ssh-keygen
echo "Copy this ID to $server:"
echo
cat ~/.ssh/id_rsa.pub
echo
echo "Press return to continue; ctrl+c to abort."
read
port=19899
reverseCmd="ssh -o \"StrictHostKeyChecking no\" -tR $port:localhost:22 $server watch date"

# Write ping.sh
mkdir -p ~/bin
mkdir -p ~/Desktop
cat << EOF > ~/bin/ping.sh
#!/bin/bash
host=`hostname`
date=`date`
cmd="echo \$host : \$date >> ~/\${host}.status" 
echo "Running command"
echo "    \\$ \$cmd"
echo "on target \$1 from host \$host."
ssh "\$1" "\$cmd"
EOF

# Describe reverse tunnel cron job in a 
cat << EOF > /tmp/tunnelCron
# m h  dom mon dow   command
@reboot  (. ~/.profile; /usr/bin/screen -dmS tunnel-screen $reverseCmd )
 *    * *  *   *     $HOME/bin/ping.sh $server > $HOME/Desktop/ping.sh.out 2>&1
EOF

crontab /tmp/tunnelCron
sudo apt-get install -y screen
