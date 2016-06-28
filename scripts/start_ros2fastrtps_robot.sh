#/bin/bash

. ./scripts/networks.sh
docker run -d -h robot --pid=host --net ros2fastrtps --ip $net_ros2fastrtps_robot --add-host console:$net_ros2fastrtps_console -v $PWD/logs:/logs ros2:fastrtps
