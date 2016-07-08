FROM ros2:base

ENV top /ros2_benchmarking
ENV comm $top/comm
ENV OSPL_URI file:///ros2_benchmarking/comm/ros2node/ospl.xml

RUN mkdir $top
ADD comm $comm
ADD docker/ros2_opensplice/*.patch /ros2_ws/src/
RUN mkdir $comm/build

RUN cd /ros2_ws/src/ && cat *.patch | patch -p0 && mv *.patch applied
RUN cd /ros2_ws && ./src/ament/ament_tools/scripts/ament.py build $AMENT_ARGS
RUN cd $comm/build && /bin/bash -c "source $ROS2_SETUP && cmake -DCOMM_ROS2_OPENSPLICE=true .. && make"

CMD cd $comm/build/scenarios && ./ros2runner ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1
