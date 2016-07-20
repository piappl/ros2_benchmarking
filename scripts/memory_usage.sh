#!/bin/bash

while true
do
    usage=`free -m | grep "buffers/cache"`
    expression=`echo $usage | sed 's/.*cache:\s*\([0-9]\+\)\s*\([0-9]\+\).*/\1\/\2/'`
    echo "scale=2; 100*$expression" | bc -l
    sleep 1
done
