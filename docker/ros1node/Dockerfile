FROM test:ros1

ENV top /r5cop_benchmarking
ENV comm $top/comm

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build
RUN cd $comm/build && cmake -DCOMM_ROS=true .. && make

CMD sleep 2 && cd $comm/build/scenarios && ./ros1runner $HOSTNAME ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1
