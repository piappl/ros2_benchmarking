#/bin/bash

. ./scripts/networks.sh
docker run -d -h robot --net opensplice --ip $net_opensplice_robot --add-host console:$net_opensplice_console -v $PWD/logs:/logs opensplice:node
