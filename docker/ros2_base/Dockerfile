FROM ubuntu:xenial

RUN cd /etc && rm localtime && ln -s /usr/share/zoneinfo/Poland localtime
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV top /ros2_benchmarking
ENV comm $top/comm
ENV RMW_IMPLEMENTATION rmw_opensplice_cpp
ENV ROS2_SETUP /ros2_ws/install/local_setup.bash
ENV OSPL_URI file:///usr/etc/opensplice/config/ospl.xml
ENV AMENT_ARGS --build-space /ros2_ws/build --install-space /ros2_ws/install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -- /ros2_ws/src
RUN mkdir $top

RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu xenial main" > /etc/apt/sources.list.d/gazebo-latest.list

RUN apt-get update
RUN apt-get install -y perl git wget build-essential cppcheck cmake libopencv-dev python-empy python3-empy python3-nose python3-pip python3-setuptools python3-vcstool qt5-default qtcreator
RUN apt-get install -y libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libopensplice64 vim openjdk-9-jre
RUN pip3 install -U setuptools

RUN mkdir -p /ros2_ws/src/applied
RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -O /ros2_ws/ros2.repos
RUN vcs import /ros2_ws/src < /ros2_ws/ros2.repos
RUN perl -i -p -e "s/#(?=.+\/(Byte|ByteMultiArray|MultiArrayDimension|MultiArrayLayout)\.msg)//" /ros2_ws/src/ros2/common_interfaces/std_msgs/CMakeLists.txt
RUN for dir in demos examples rclpy realtime_support ros1_bridge rmw_connext system_tests tlsf ../eProsima; do touch /ros2_ws/src/ros2/$dir/AMENT_IGNORE; done
ADD comm/ros2node/messages /ros2_ws/src/ros2/common_interfaces/messages
RUN /ros2_ws/src/ament/ament_tools/scripts/ament.py build $AMENT_ARGS

