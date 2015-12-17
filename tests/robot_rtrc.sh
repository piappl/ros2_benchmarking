#!/bin/sh

cd /home/mother/mother/bin
sleep 5
./dummyscout --beacon-addr=`getent hosts console | awk '{ print $1 }'` --commtype=0

