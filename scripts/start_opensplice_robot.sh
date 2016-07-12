#/bin/bash

. ./scripts/networks.sh
cmd="source \$release && OSPL_ERRORFILE='<stderr>' OSPL_INFOFILE='<stderr>' OSPL_VERBOSITY=WARNING && cd \$comm/build/scenarios && ./ddsrunner rosrobot.ini >/logs/robot.txt 2>&1"
docker run -d -h robot --net opensplice --ip $net_opensplice_robot --add-host console:$net_opensplice_console -v $PWD/logs:/logs opensplice:node bash -c "$cmd"
