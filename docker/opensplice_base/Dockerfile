FROM ubuntu:latest

ENV release /opensplice/install/HDE/x86_64.linux-dev/release.com
ENV envs /opensplice/envs-x86_64.linux-dev.sh

RUN cd /etc && rm localtime && ln -s /usr/share/zoneinfo/Poland localtime
RUN apt-get update && apt-get install -y perl git wget build-essential cppcheck cmake vim gawk doxygen flex bison qt5-default qtcreator

RUN git clone https://github.com/PrismTech/opensplice.git
ADD docker/opensplice_base/*.patch /opensplice/
RUN cd /opensplice && patch -p0 < DCPS_ISO_Cpp.mpc.patch
RUN cd /opensplice/src/api/dcps/isocpp/include/ && patch -p0 < /opensplice/macros.hpp.patch
RUN cd /opensplice/src/api/dcps/isocpp/include/spec && patch -p0 < /opensplice/macros.hpp.patch
RUN cd /opensplice && /bin/bash -c "source ./configure x86_64.linux-dev && make && make install"
RUN sed -i 's/@@INSTALLDIR@@/\/opensplice\/install/' $release

