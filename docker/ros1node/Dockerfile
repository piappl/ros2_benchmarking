FROM test:ros1

ENV top /ros2_benchmarking
ENV comm $top/comm

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build
RUN touch $comm/ros2node/messages/CATKIN_IGNORE
RUN cd $comm/build && /bin/bash -c "source /opt/ros/jade/setup.bash && cmake -DCOMM_ROS=true .. && make"

CMD sleep 2 && cd $comm/build/scenarios && ./ros1runner ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1
