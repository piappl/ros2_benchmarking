#/bin/bash

. ./scripts/networks.sh
docker run -d -h console --pid=host --net ros2connext --ip $net_ros2connext_console --add-host robot:$net_ros2connext_robot -v $PWD/logs:/logs ros2:connext
