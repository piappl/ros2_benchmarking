#!/bin/bash

cmd="source \$ROS2_SETUP; source \$CONNEXT_SETUP; qtcreator /ros2_benchmarking/comm/CMakeLists.txt"
export XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
docker run -d -h qtcreator -v /tmp/.X11-unix:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth -e XAUTHORITY=/tmp/.docker.xauth -e DISPLAY $1 /bin/bash -c "$cmd"
