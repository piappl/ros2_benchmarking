FROM ros2:base

ADD comm $comm
ADD docker/ros2_fastrtps/*.patch /ros2_ws/src/
RUN mkdir $comm/build
RUN rm /ros2_ws/src/eProsima/AMENT_IGNORE
RUN cd /ros2_ws/src/ && cat *.patch | patch -p0 && mv *.patch applied
RUN cd /ros2_ws && ./src/ament/ament_tools/scripts/ament.py build --force-ament-cmake-configure $AMENT_ARGS
RUN cd $comm/build && /bin/bash -c "source $ROS2_SETUP && cmake -DCOMM_ROS2_FASTRTPS=true .. && make"
