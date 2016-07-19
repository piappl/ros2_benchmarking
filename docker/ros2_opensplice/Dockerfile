FROM ros2:base

ENV OSPL_URI file:///ros2_benchmarking/comm/ros2node/ospl.xml
ADD comm $comm
RUN mkdir $comm/build
RUN cd $comm/build && /bin/bash -c "source $ROS2_SETUP && cmake -DCOMM_ROS2_OPENSPLICE=true .. && make"
