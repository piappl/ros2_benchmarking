#/bin/bash

. ./scripts/networks.sh
docker run -d -h robot --net ros2opensplice --ip $net_ros2opensplice_robot --add-host console:$net_ros2opensplice_console -v $PWD/logs:/logs ros2:opensplice
