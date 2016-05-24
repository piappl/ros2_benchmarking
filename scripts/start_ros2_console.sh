#/bin/bash

. ./scripts/networks.sh
docker run -d -h console --net ros2 --ip $net_ros2_console --add-host robot:$net_ros2_robot -v $PWD/logs:/logs test:ros2node
