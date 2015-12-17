#!/bin/sh

#sudo sh -c "echo '172.17.0.3 console ' >> /etc/hosts"

roscore &
cd /home/mother/mother/bin
./dummyscout --commtype=1 
