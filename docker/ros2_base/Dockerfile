FROM ubuntu:trusty

RUN cd /etc && rm localtime && ln -s /usr/share/zoneinfo/Poland localtime
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV ROS2_SETUP /ros2_ws/install/local_setup.bash
ENV AMENT_ARGS --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list

RUN apt-get update
RUN sudo apt-get install -y perl git wget build-essential cppcheck cmake libopencv-dev python-empy python3-empy python3-nose python3-pip python3-setuptools python3-vcstool qt5-default qtcreator
RUN sudo apt-get install -y libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libopensplice64 vim openjdk-7-jre
RUN sudo pip3 install -U setuptools

RUN mkdir -p /ros2_ws/src/applied
RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -O /ros2_ws/ros2.repos
RUN C=2fefcf42d30da417ed5abdb6b02d4668df232b54 && grep -q $C /ros2_ws/ros2.repos && sed -i s/$C/834f01c5b295f23e38dbc4241e3ead66199cc0d8/ /ros2_ws/ros2.repos
RUN C=c683f60c271c3e6b4ffbbd04d010e1035ef1e987 && grep -q $C /ros2_ws/ros2.repos && sed -i s/$C/v1.2.0/ /ros2_ws/ros2.repos
RUN vcs import /ros2_ws/src < /ros2_ws/ros2.repos
RUN perl -i -p -e "s/#(?=.+\/(Byte|ByteMultiArray|MultiArrayDimension|MultiArrayLayout)\.msg)//" /ros2_ws/src/ros2/common_interfaces/std_msgs/CMakeLists.txt
RUN for dir in demos examples rclpy realtime_support ros1_bridge system_tests tlsf; do touch /ros2_ws/src/ros2/$dir/AMENT_IGNORE; done
ADD comm/ros2node/messages /ros2_ws/src/ros2/common_interfaces/messages
ADD docker/ros2_base/*.patch /ros2_ws/src/
RUN cd /ros2_ws/src/ && cat *.patch | patch -p0 && mv *.patch applied
RUN cd /ros2_ws && ./src/ament/ament_tools/scripts/ament.py build $AMENT_ARGS

