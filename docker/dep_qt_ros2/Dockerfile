FROM mother:dds

#Get some sources in right places
#There should be folders with canfestival, marble library and mapwidget in this directory
ADD marblelib /src/marblelib
ADD map /src/map

#Install other mother dependencies
RUN apt-get install -y libpopt-dev libxv-dev libspeex-dev libnl-3-dev libnl-genl-3-dev libftgl-dev freeglut3-dev libnfc-dev libftdi-dev libftdi1 mercurial
RUN ln -s /usr/lib/x86_64-linux-gnu/libftdi.so /usr/lib/x86_64-linux-gnu/libftdi1.so

#Build and install canfestival
RUN cd /src && hg clone http://dev.automforge.net/CanFestival-3/ canfestival
RUN cd /src/canfestival && ./configure --prefix=/usr && make  && make install

#Build marble
RUN mkdir -p /src/marblelib/build
RUN cd /src/marblelib/build && cmake -DCMAKE_BUILD_TYPE=Debug -DQTONLY=1 -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local/lib/cmake -DBUILD_WITH_DBUS=0 -DQT_NO_DBUS=1 -DBUILD_MARBLE_APPS=OFF ../
RUN cd /src/marblelib/build && make -j12  && make install

#Build mapwidget
RUN mkdir -p /src/map/build
RUN cd /src/map/build && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local/lib/cmake -DCMAKE_MODULE_PATH=/usr/local/share/marble/cmake ../
RUN cd /src/map/build && make -j12 && make install

RUN useradd -m mother
RUN echo "mother ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers


#Get ROS2
USER mother
RUN mkdir -p /home/mother/ros2_ws/src && cd /home/mother/ros2_ws && wget https://raw.githubusercontent.com/ros2/ros2/release-alpha2/ros2.repos && vcs import /home/mother/ros2_ws/src < ros2.repos


#Quickfix for ros1_bridge problem, two options, remove roslz import from rosbag or don't build ros1_bridge:
#RUN sed -i 's/import roslz4/#import roslz4/' $ROS_ROOT/../../lib/python2.7/dist-packages/rosbag/bag.py
#RUN cd /home/ros/ros2_ws && touch src/ros2/ros1_bridge/AMENT_IGNORE
#RUN cd /home/ros/ros2_ws && src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
#RUN cd /home/ros/ros2_ws && rm src/ros2/ros1_bridge/AMENT_IGNORE
USER root
RUN sed -i 's/import roslz4/#import roslz4/' /opt/ros/jade/lib/python2.7/dist-packages/rosbag/bag.py
USER mother
RUN cd /home/mother/ros2_ws && src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install

RUN echo "source /opt/ros/jade/setup.bash" >> /home/mother/.bashrc
RUN echo "export OSPL_URI=file:///usr/etc/opensplice/config/ospl.xml" >> /home/mother/.bashrc
RUN echo "source /home/mother/ros2_ws/install/local_setup.bash" >> /home/mother/.bashrc

RUN cp  /home/mother/ros2_ws/src/ros2/rmw_opensplice/rosidl_typesupport_opensplice_cpp/include/rosidl_typesupport_opensplice_cpp/impl/rosidl_generator_cpp/* /home/mother/ros2_ws/install/include/rosidl_generator_cpp/


ENV DISPLAY :0
ENV QT_X11_NO_MITSHM 1
CMD /usr/bin/terminator
#CMD bash
