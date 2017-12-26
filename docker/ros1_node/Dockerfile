FROM ros1:base

ENV top /ros2_benchmarking
ENV comm $top/comm

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build
RUN touch $comm/ros2node/messages/CATKIN_IGNORE
RUN cd $comm/build && /bin/bash -c "source /opt/ros/lunar/setup.bash && cmake -DCOMM_ROS=true .. && make"
