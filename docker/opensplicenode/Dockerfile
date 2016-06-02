FROM test:opensplice

ENV top /r5cop_benchmarking
ENV comm $top/comm
ENV release /opensplice/install/HDE/x86_64.linux-dev/release.com
ENV qt5cmake /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/data/cmake
ENV envs /opensplice/envs-x86_64.linux-dev.sh

RUN mkdir $top
ADD comm $comm
RUN mkdir $comm/build

RUN sed -i 's/@@INSTALLDIR@@/\/opensplice\/install/' $release
RUN cd $comm/build && /bin/bash -c "source $release && source $envs && cmake -DCMAKE_PREFIX_PATH=$qt5cmake -DCOMM_DDS=true .. && make"

CMD cd $comm/build/scenarios && sleep 2 && /bin/bash -c "source $release && ./ddsrunner ros${HOSTNAME}.ini >/logs/$HOSTNAME.txt 2>&1"
