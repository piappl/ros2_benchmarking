FROM ubuntu:xenial

RUN cd /etc && rm localtime && ln -s /usr/share/zoneinfo/Poland localtime
RUN apt-get update 
RUN apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV top /ros2_benchmarking
ENV comm $top/comm

ENV RMW_IMPLEMENTATION rmw_opensplice_cpp
ENV ROS2_SETUP /ros2_ws/install/local_setup.bash
ENV OSPL_URI file:///usr/etc/opensplice/config/ospl.xml
ENV AMENT_ARGS --build-space /ros2_ws/build --install-space /ros2_ws/install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -- /ros2_ws/src
RUN mkdir $top

RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu xenial main" > /etc/apt/sources.list.d/gazebo-latest.list

RUN apt-get update
#build essentials
RUN apt-get install -y cmake gcc wget git
RUN apt-get install -y wget build-essential cppcheck libopencv-dev libpoco-dev libpocofoundation9v5 libpocofoundation9v5-dbg

RUN apt-get install -y qt5-default qtcreator
RUN apt-get install -y libtinyxml-dev libeigen3-dev uncrustify
RUN apt-get install -y clang-format pydocstyle pyflakes  qtbase5-dev

#dependencies for RViz
RUN apt-get install -y libcurl4-openssl-dev libqt5core5a libqt5gui5 libqt5opengl5 libqt5widgets5 libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev

#pyhton requirements
RUN apt-get install -y python-empy python3-empy python3-dev python3-nose python3-pip python3-setuptools python3-vcstool python3-yaml python3-coverage python3-mock python3-pep8 python3-pyparsing
RUN pip3 install -U argcomplete
RUN pip3 install -U flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest pytest-cov pytest-runner

#Fastrtps and opensplice requirements
RUN apt-get install -y libasio-dev libtinyxml2-dev libopensplice67


RUN apt-get install -y vim openjdk-9-jre


RUN mkdir -p /ros2_ws/src/applied
RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos -O /ros2_ws/ros2.repos
RUN vcs import /ros2_ws/src < /ros2_ws/ros2.repos

RUN for dir in demos examples rclpy realtime_support ros1_bridge rviz rmw_connext system_tests tlsf; do touch /ros2_ws/src/ros2/$dir/AMENT_IGNORE; done
ADD comm/ros2node/messages /ros2_ws/src/ros2/common_interfaces/messages
RUN /ros2_ws/src/ament/ament_tools/scripts/ament.py build $AMENT_ARGS

