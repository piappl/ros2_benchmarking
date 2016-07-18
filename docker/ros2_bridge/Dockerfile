FROM ros2:base

RUN apt-get install -y ros-kinetic-roscpp ros-kinetic-rosmsg ros-kinetic-catkin ros-kinetic-std-msgs

ENV top /ros2_benchmarking
ENV comm $top/comm
ENV build $comm/ros1node/messages/build
RUN mkdir $top
ADD comm $comm
RUN mkdir $build
RUN cd $build && /bin/bash -c "source /opt/ros/kinetic/setup.bash && cmake -DCMAKE_INSTALL_PREFIX:PATH=/opt/ros/kinetic/ .. && make && make install"

RUN rm -rf /ros2_ws/build /ros2_ws/install
RUN touch /ros2_ws/src/ros2/rmw_connext/AMENT_IGNORE
RUN touch /ros2_ws/src/eProsima/AMENT_IGNORE
RUN cd /ros2_ws && ./src/ament/ament_tools/scripts/ament.py build $AMENT_ARGS
RUN rm /ros2_ws/src/ros2/ros1_bridge/AMENT_IGNORE
RUN cd /ros2_ws && bash -c "source /opt/ros/kinetic/setup.bash && ./src/ament/ament_tools/scripts/ament.py build -j1 --only ros1_bridge $AMENT_ARGS"

