#/bin/bash

. ./scripts/networks.sh
cmd="source /ros2_ws/install/setup.sh && cd \$comm/build/scenarios && ./ros2runner rosrobot.ini >/logs/robot.txt 2>&1"
docker run -d -h robot --pid=host --net ros2fastrtps --ip $net_ros2fastrtps_robot --add-host console:$net_ros2fastrtps_console -v $PWD/logs:/logs ros2:fastrtps bash -c "$cmd"
