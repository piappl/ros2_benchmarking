FROM mother:dep_qt_ros

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
ENV LD_LIBRARY_PATH /opt/ros/jade/lib/x86_64-linux-gnu:/opt/ros/jade/lib:/home/mother/mother/bin:/usr/local/lib/x86_64-linux-gnu
ENV CPATH /opt/ros/jade/include
ENV PATH /opt/ros/jade/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/mother/mother/bin
ENV QT_X11_NO_MITSHM 1
ENV HOME /home/mother
ENV ROS_DISTRO jade
ENV PYTHONPATH /opt/ros/jade/lib/python2.7/dist-packages
ENV PKG_CONFIG_PATH /opt/ros/jade/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/jade/lib/pkgconfig
ENV CMAKE_PREFIX_PATH /opt/ros/jade
ENV DISPLAY :0
ENV ROS_ETC_DIR /opt/ros/jade/etc/ros


#Build mother
RUN cd /home/mother/mother && /usr/local/bin/qmake -r -spec linux-g++ CONFIG+=debug DEFINES+=ROS_ENABLED  DEFINES+=TURN_LOGGING_ON INCLUDEPATH+=/usr/include/canfestival INCLUDEPATH+=/opt/ros/jade/include  CONFIG+=BUILD_PSOR CONFIG+=BUILD_SCOUT CONFIG+=BUILD_ROS
RUN cd /home/mother/mother && make -j12

CMD /usr/bin/terminator
