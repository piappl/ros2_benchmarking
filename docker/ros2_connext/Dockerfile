FROM ros2:base

ENV CONNEXT_SETUP=/opt/rti_connext_dds/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
ENV RTI_LICENSE_FILE /opt/rti_connext_dds/rti_license.dat
ADD comm $comm
ADD comm/rti_connext_dds /opt/rti_connext_dds
ADD docker/ros2_connext/*.patch /ros2_ws/src/
RUN mkdir $comm/build
RUN rm /ros2_ws/src/ros2/rmw_connext/AMENT_IGNORE
RUN cd /ros2_ws/src/ && cat *.patch | patch -p0 && mv *.patch applied
RUN cd /ros2_ws && bash -c "source $CONNEXT_SETUP && ./src/ament/ament_tools/scripts/ament.py build --force-ament-cmake-configure $AMENT_ARGS"
RUN cd $comm/build && /bin/bash -c "source $ROS2_SETUP && source $CONNEXT_SETUP && cmake -DCOMM_ROS2_CONNEXT=true .. && make"
