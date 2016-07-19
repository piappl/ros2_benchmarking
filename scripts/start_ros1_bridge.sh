#/bin/bash

. scripts/networks.sh
master_uri="ROS_MASTER_URI=http://$net_ros1_master:$net_ros1_port"
cmd="cd /ros2_ws/install/bin && source /opt/ros/kinetic/setup.bash && OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml ./dynamic_bridge >/logs/bridge.txt 2>&1"
docker run -d -h master --net ros1 --ip $net_ros1_bridge -e $master_uri --add-host console:$net_ros1_console --add-host robot:$net_ros1_ros2_robot --add-host master:$net_ros1_master -v $PWD/logs:/logs ros2:bridge bash -c "$cmd"
