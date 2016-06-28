FROM opensplice:base

ENV top /ros2_benchmarking
ENV comm $top/comm
ENV qt5cmake /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/data/cmake

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build

RUN cd $comm/build && /bin/bash -c "source $release && source $envs && cmake -DCMAKE_PREFIX_PATH=$qt5cmake -DCOMM_OPENSPLICE=true .. && make"

CMD cd $comm/build/scenarios && sleep 2 && /bin/bash -c "source $release && ./ddsrunner ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1"
