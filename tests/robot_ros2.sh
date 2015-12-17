#!/bin/bash

#sudo sh -c "echo '172.17.0.3   console ' >> /etc/hosts"
#roscore &

export OSPL_VERBOSITY=2
export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml
export OSPL_TMPL_PATH=/usr/etc/opensplice/idlpp

source /home/mother/.bashrc
cd /home/mother/mother/bin
./dummyscout --commtype=3 
