#/bin/bash

. ./scripts/networks.sh
docker run -d -h robot --net ros2 --ip $net_ros2_robot --add-host console:$net_ros2_console -v $PWD/logs:/logs test:ros2node
