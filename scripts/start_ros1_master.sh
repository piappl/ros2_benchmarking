#/bin/bash

. scripts/networks.sh
master_uri="ROS_MASTER_URI=http://$net_ros1_master:$net_ros1_port"
docker run -d -h master --net ros1 --ip $net_ros1_master -e $master_uri --add-host console:$net_ros1_console --add-host robot:$net_ros1_robot test:ros1 roscore -p $net_ros1_port
