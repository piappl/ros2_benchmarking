#/bin/bash

. ./scripts/networks.sh
cmd="source \$release && cd \$comm/build/scenarios && OSPL_ERRORFILE='<stderr>' OSPL_INFOFILE='<stderr>' OSPL_VERBOSITY=WARNING ./ddsrunner rosconsole.ini >/logs/console.txt 2>&1"
docker run -d -h console --net opensplice --ip $net_opensplice_console --add-host robot:$net_opensplice_robot -v $PWD/logs:/logs opensplice:node bash -c "$cmd"
