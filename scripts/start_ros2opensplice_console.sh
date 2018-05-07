#/bin/bash

. ./scripts/networks.sh
cmd="source /ros2_ws/install/setup.sh && cd \$comm/build/scenarios && OSPL_ERRORFILE='<stderr>' OSPL_INFOFILE='<stderr>' OSPL_VERBOSITY=WARNING ./ros2runner rosconsole.ini >/logs/console.txt 2>&1"
docker run -d -h console --net ros2opensplice --ip $net_ros2opensplice_console --add-host robot:$net_ros2opensplice_robot -v $PWD/logs:/logs ros2:opensplice bash -c "$cmd"
