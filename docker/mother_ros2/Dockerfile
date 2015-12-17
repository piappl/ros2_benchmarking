FROM mother:dep_qt_ros2

USER root

#Get some sources in right places
ADD mother /home/mother/mother

RUN chown -R mother:mother /home/mother/mother

RUN echo '/home/mother/mother/bin' > /etc/ld.so.conf.d/mother.conf
RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/mother.conf
RUN ldconfig

USER mother
ENV ROS_ROOT /opt/ros/jade/share/ros
ENV ROS_PACKAGE_PATH /opt/ros/jade/share:/opt/ros/jade/stacks
ENV ROS_MASTER_URI http://localhost:11311
ENV LD_LIBRARY_PATH /usr/lib:/home/mother/ros2_ws/install/lib:/opt/ros/jade/lib/x86_64-linux-gnu:/opt/ros/jade/lib:/home/mother/mother/bin:/usr/local/lib/x86_64-linux-gnu
ENV CPATH /opt/ros/jade/include
ENV PATH /home/mother/ros2_ws/install/bin:/opt/ros/jade/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/mother/mother/bin
ENV QT_X11_NO_MITSHM 1
ENV HOME /home/mother
ENV ROS_DISTRO jade
ENV PYTHONPATH /home/mother/ros2_ws/install/lib/python3.4/site-packages:/opt/ros/jade/lib/python2.7/dist-packages
ENV PKG_CONFIG_PATH /opt/ros/jade/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/jade/lib/pkgconfig
ENV CMAKE_PREFIX_PATH /opt/ros/jade
ENV DISPLAY :0
ENV ROS_ETC_DIR /opt/ros/jade/etc/ros


RUN cp -r /home/mother/mother/sandbox/robot_information_msgs  /home/mother/ros2_ws/src/ros2/common_interfaces
RUN cd /home/mother/ros2_ws && rm -rf build/ install/
RUN cd /home/mother/ros2_ws && src/ament/ament_tools/scripts/ament.py build  --symlink-install
RUN cp  /home/mother/ros2_ws/src/ros2/rmw_opensplice/rosidl_typesupport_opensplice_cpp/include/rosidl_typesupport_opensplice_cpp/impl/rosidl_generator_cpp/* /home/mother/ros2_ws/install/include/rosidl_generator_cpp/

#Build mother
RUN cd /home/mother/mother && /usr/local/bin/qmake -r -spec linux-g++ CONFIG+=debug DEFINES+=ROS2_ENABLED DEFINES+=TURN_LOGGING_ON INCLUDEPATH+=/usr/include/canfestival INCLUDEPATH+=/opt/ros/jade/include INCLUDEPATH+=/home/mother/ros2_ws/install/include  CONFIG+=BUILD_PSOR CONFIG+=BUILD_SCOUT CONFIG+=BUILD_ROS2
RUN cd /home/mother/mother && make -j12

CMD /usr/bin/terminator
