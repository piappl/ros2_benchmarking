#/bin/bash

. ./scripts/networks.sh
docker run -d -h console --net opensplice --ip $net_opensplice_console --add-host robot:$net_opensplice_robot -v $PWD/logs:/logs test:opensplicenode
