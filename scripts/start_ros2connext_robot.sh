#/bin/bash

. ./scripts/networks.sh
cmd="cd \$comm/build/scenarios && ./ros2runner rosrobot.ini >/logs/robot.txt 2>&1"
docker run -d -h robot --pid=host --net ros2connext --ip $net_ros2connext_robot --add-host console:$net_ros2connext_console -v $PWD/logs:/logs ros2:connext bash -c "$cmd"
