#!/bin/bash

while true
do
    idle=`top -bn2 | grep "Cpu(s)" | sed 1d | sed "s/.*, \([0-9,]\+\) id.*/\\1/" | sed "s/,/./"`
    echo "scale=2; 100-$idle" | bc -l
    sleep 1
done
