#!/bin/bash

. scripts/networks.sh

if ! docker network ls | egrep -q "\bros1\b"
then
    docker network create --subnet=$net_ros1 ros1
fi

if ! docker network ls | egrep -q "\bros2opensplice\b"
then
    docker network create --subnet=$net_ros2opensplice ros2opensplice
fi

if ! docker network ls | egrep -q "\bros2fastrtps\b"
then
    docker network create --subnet=$net_ros2fastrtps ros2fastrtps
fi

if ! docker network ls | egrep -q "\bopensplice\b"
then
    docker network create --subnet=$net_opensplice opensplice
fi

./scripts/remove_image.sh $1

docker build -t $1 -f docker/`echo -n $1 | sed s/:/_/`/Dockerfile .
