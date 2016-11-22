#!/bin/bash
>&2 echo "This file is $0."
thisDir=`dirname $0`
>&2 echo "This dir is $thisDir."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH $thisDir/hardwareDriver
