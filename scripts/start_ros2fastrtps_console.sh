#/bin/bash

. ./scripts/networks.sh
docker run -d -h console --pid=host --net ros2fastrtps --ip $net_ros2fastrtps_console --add-host robot:$net_ros2fastrtps_robot -v $PWD/logs:/logs ros2:fastrtps
