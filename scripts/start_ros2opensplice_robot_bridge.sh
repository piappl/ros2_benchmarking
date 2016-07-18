#/bin/bash

. ./scripts/networks.sh
cmd="cd \$comm/build/scenarios && OSPL_ERRORFILE='<stderr>' OSPL_INFOFILE='<stderr>' OSPL_VERBOSITY=WARNING ./ros2runner rosrobot.ini >/logs/robot.txt 2>&1"
docker run -d -h robot --net ros1 --ip $net_ros1_ros2_robot --add-host console:$net_ros1_console -v $PWD/logs:/logs ros2:opensplice bash -c "$cmd"
