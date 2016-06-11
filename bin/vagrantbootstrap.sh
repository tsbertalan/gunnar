#!/usr/bin/env bash

# Some commands need to be run *not* as root, but as the local user.
function runAsUser {
    sudo su vagrant --login --shell=/bin/bash --command "$1"
}
runAsUser "mkdir -p /home/vagrant/sketchbook"
runAsUser "ln -s /vagrant /home/vagrant/sketchbook/gunnar"
runAsUser "/bin/bash ~/sketchbook/gunnar/bin/bootstrap.sh"
