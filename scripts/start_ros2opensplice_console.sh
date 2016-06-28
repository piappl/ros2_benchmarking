#/bin/bash

. ./scripts/networks.sh
docker run -d -h console --net ros2opensplice --ip $net_ros2opensplice_console --add-host robot:$net_ros2opensplice_robot -v $PWD/logs:/logs ros2:opensplice
