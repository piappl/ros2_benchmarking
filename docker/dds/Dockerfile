FROM mother:nvidia_qt_ros

#We will install DDS (opensplice) from osrf gazebo repo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN apt-get update

#Install some ROS2 dependencies
RUN apt-get install -y git wget build-essential cppcheck cmake libopencv-dev python-empy python3-empy python3-setuptools python3-nose python3-pip python3-vcstool ros-jade-rosbag tcpdump
RUN sudo pip3 install -U setuptools

#Install DDS
RUN apt-get install -y libopensplice64 

CMD bash
