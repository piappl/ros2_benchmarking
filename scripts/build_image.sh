#!/bin/bash

if [ ! -e comm ]
then
    echo "Run this script from the top directory of r5cop_banchmarking project"
    exit 1
fi

. scripts/networks.sh

if ! docker network ls | egrep -q "\bros1\b"
then
    docker network create --subnet=$net_ros1 ros1
fi

if ! docker network ls | egrep -q "\bros2\b"
then
    docker network create --subnet=$net_ros2 ros2
fi

if ! docker network ls | egrep -q "\bopensplice\b"
then
    docker network create --subnet=$net_opensplice opensplice
fi

./scripts/remove_image.sh $1

docker build -t test:$1 -f docker/$1/Dockerfile .
