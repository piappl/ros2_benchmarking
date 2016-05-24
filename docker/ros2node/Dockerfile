FROM test:ros2

ENV top /r5cop_benchmarking
ENV comm $top/comm

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build
RUN cd $comm/build && cmake -DCOMM_ROS2=true .. && make

CMD sleep 2 && cd $comm/build/scenarios && ./ros2runner $HOSTNAME ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1
