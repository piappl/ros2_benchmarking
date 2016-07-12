#/bin/bash

. ./scripts/networks.sh
cmd="source \$release && OSPL_ERRORFILE='<stderr>' OSPL_INFOFILE='<stderr>' OSPL_VERBOSITY=WARNING && cd \$comm/build/scenarios && ./ddsrunner rosconsole.ini >/logs/console.txt 2>&1"
docker run -d -h console --net opensplice --ip $net_opensplice_console --add-host robot:$net_opensplice_robot -v $PWD/logs:/logs opensplice:node bash -c "$cmd"
