FROM ubuntu:trusty

RUN cd /etc && rm localtime && ln -s /usr/share/zoneinfo/Poland localtime
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV OSPL_URI file:///usr/etc/opensplice/config/ospl.xml

RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list

RUN apt-get update
RUN sudo apt-get install -y perl git wget build-essential cppcheck cmake libopencv-dev python-empy python3-empy python3-nose python3-pip python3-setuptools python3-vcstool qtcreator
RUN sudo apt-get install -y libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libopensplice64
RUN sudo pip3 install -U setuptools
RUN apt-get build-dep -y qt5-default

RUN mkdir /src
RUN wget http://download.qt.io/official_releases/qt/5.5/5.5.1/single/qt-everywhere-opensource-src-5.5.1.tar.gz -O - | tar -xzC /src
RUN mv /src/qt-everywhere-opensource-src-5.5.1 /src/qt
RUN cd /src/qt && ./configure -confirm-license -opensource -nomake examples -nomake tests -no-compile-examples -prefix "/usr/local"
RUN cd /src/qt && make -j12 all && make install

RUN mkdir -p /ros2_ws/src
RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -O /ros2_ws/ros2.repos
RUN vcs import /ros2_ws/src < /ros2_ws/ros2.repos
RUN perl -i -p -e "s/#(?=.+\/(Byte|ByteMultiArray|MultiArrayDimension|MultiArrayLayout)\.msg)//" /ros2_ws/src/ros2/common_interfaces/std_msgs/CMakeLists.txt
RUN touch /ros2_ws/src/ros2/ros1_bridge/AMENT_IGNORE
ADD ros2_psor_messages /ros2_ws/src/ros2/common_interfaces/robot_information_msgs
RUN cd /ros2_ws && ./src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
