#/bin/bash

. ./scripts/networks.sh
docker run -d -h robot --pid=host --net ros2connext --ip $net_ros2connext_robot --add-host console:$net_ros2connext_console -v $PWD/logs:/logs ros2:connext
