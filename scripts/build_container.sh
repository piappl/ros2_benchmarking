#!/bin/bash

if [ ! -e comm ]
then
    echo "Run this script from the top directory of r5cop_banchmarking project"
    exit 1
fi

if [ ! -e ros2_psor_messages ]
then
    echo "Clone ros2_psor_messages project to the current directory:"
    echo ""
    echo "git clone ssh://$USER@10.0.9.90:29418/ros2_psor_messages"
    echo ""
    exit 1
fi

. scripts/networks.sh

if [ 1 -eq `docker network ls -f name=ros1 | wc -l` ]
then
    docker network create --subnet=$net_ros1 ros1
fi

if [ 1 -eq `docker network ls -f name=ros2 | wc -l` ]
then
    docker network create --subnet=$net_ros2 ros2
fi

if [ 1 -ne `docker images test:$1 | wc -l` ]
then
    docker rmi test:$1
fi

docker build -t test:$1 -f docker/$1/Dockerfile .
