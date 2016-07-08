FROM ros2:base

ENV CONNEXT_SETUP=/opt/rti_connext_dds/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
ENV RTI_LICENSE_FILE /opt/rti_connext_dds/rti_license.dat
ENV top /ros2_benchmarking
ENV comm $top/comm

RUN mkdir $top
ADD comm $comm
ADD comm/rti_connext_dds /opt/rti_connext_dds
RUN mkdir $comm/build

RUN touch /ros2_ws/src/ros2/rmw_connext/*/CMakeLists.txt
RUN cd /ros2_ws && bash -c "source $CONNEXT_SETUP && ./src/ament/ament_tools/scripts/ament.py build $AMENT_ARGS"
RUN cd $comm/build && /bin/bash -c "source $ROS2_SETUP && source $CONNEXT_SETUP && cmake -DCOMM_ROS2_CONNEXT=true .. && make"

CMD cd $comm/build/scenarios && ./ros2runner ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1
