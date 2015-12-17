#!/bin/sh


#sudo sh -c "echo '172.17.0.2    robot' >>/etc/hosts"
cd /home/mother/mother/bin
export ROS_MASTER_URI=http://robot:11311 
./taskhostprocess configtask 
