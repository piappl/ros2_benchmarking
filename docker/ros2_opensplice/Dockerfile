FROM ros2:base

ENV top /ros2_benchmarking
ENV comm $top/comm
ENV OSPL_URI file:///ros2_benchmarking/comm/ros2node/ospl.xml

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build
RUN cd $comm/build && /bin/bash -c "source $LOCAL_SETUP && cmake -DCOMM_ROS2_OPENSPLICE=true .. && make"

CMD sleep 2 && cd $comm/build/scenarios && ./ros2runner ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1
