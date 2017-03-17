FROM ros2:base

RUN apt-get install -y ros-kinetic-roscpp ros-kinetic-rosmsg ros-kinetic-catkin ros-kinetic-std-msgs

ENV ROS_SETUP /opt/ros/kinetic/setup.bash
ENV build $comm/ros1node/messages/build
ADD comm $comm
RUN mkdir $build
RUN cd $build && /bin/bash -c "source $ROS_SETUP && cmake -DCMAKE_INSTALL_PREFIX:PATH=/opt/ros/kinetic/ .. && make && make install"
RUN cd /ros2_ws/src/ros2/ros1_bridge && rm AMENT_IGNORE 
RUN cat /ros2_ws/src/ros2/ros1_bridge/ros1_bridge/__init__.py
RUN cd /ros2_ws && bash -c "source $ROS_SETUP && ./src/ament/ament_tools/scripts/ament.py build -j1 --only ros1_bridge $AMENT_ARGS"
