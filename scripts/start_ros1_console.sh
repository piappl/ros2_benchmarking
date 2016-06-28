#/bin/bash

. ./scripts/networks.sh
master_uri="ROS_MASTER_URI=http://$net_ros1_master:$net_ros1_port"
docker run -d -h console --net ros1 --ip $net_ros1_console --add-host robot:$net_ros1_robot -e $master_uri -v $PWD/logs:/logs ros1:node
