#!/bin/bash

case "$1" in
    opensplice) cmd="source \$release && source \$envs && qtcreator /r5cop_benchmarking/comm/CMakeLists.txt" ;;
    ros1) cmd="qtcreator /r5cop_benchmarking/comm/CMakeLists.txt" ;;
    ros2) cmd="qtcreator /r5cop_benchmarking/comm/CMakeLists.txt" ;;
    *) echo "Unknown container: $1" && exit 1
esac

export XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
docker run -d -h qtcreator -v /tmp/.X11-unix:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth -v /home/rkozik/r5cop_benchmarking:/r5cop_benchmarking -e XAUTHORITY=/tmp/.docker.xauth -e DISPLAY test:$1 /bin/bash -c "$cmd"
