#/bin/bash

. ./scripts/networks.sh
cmd="cd \$comm/build/scenarios && ./ros2runner rosrobot.ini >/logs/robot.txt 2>&1"
docker run -d -h robot --net ros2opensplice --ip $net_ros2opensplice_robot --add-host console:$net_ros2opensplice_console -v $PWD/logs:/logs ros2:opensplice bash -c "$cmd"
