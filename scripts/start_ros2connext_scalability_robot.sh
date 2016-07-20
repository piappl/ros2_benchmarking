#/bin/bash

. ./scripts/networks.sh
cmd="cd \$comm/build/scenarios && ./ros2runner rosrobot.ini >/logs/robot-${1}.txt 2>&1"
docker run -d -h robot --net ros2connext --ip ${net_ros2connext_prefix}.$1 --add-host console:$net_ros2connext_console -v $PWD/logs:/logs ros2:connext bash -c "$cmd"
