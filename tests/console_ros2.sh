#!/bin/bash


#sudo sh -c "echo '172.17.0.2    robot' >>/etc/hosts"
#export ROS_MASTER_URI=http://172.17.0.2:11311 



export OSPL_VERBOSITY=2
export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml
export OSPL_TMPL_PATH=/usr/etc/opensplice/idlpp


source /home/mother/.bashrc
cd /home/mother/mother/bin
./taskhostprocess configtask 
