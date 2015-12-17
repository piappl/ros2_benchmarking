FROM mother:nvidia_qt

#Get some sources in right places
#There should be folders with canfestival, marble library and mapwidget in this directory
ADD marblelib /src/marblelib
ADD map /src/map

#Install other mother dependencies
RUN apt-get install -y libpopt-dev libxv-dev libspeex-dev libnl-3-dev libnl-genl-3-dev libftgl-dev freeglut3-dev libnfc-dev libftdi-dev libftdi1 mercurial cmake
RUN ln -s /usr/lib/x86_64-linux-gnu/libftdi.so /usr/lib/x86_64-linux-gnu/libftdi1.so

#Build and install canfestival
RUN cd /src && hg clone http://dev.automforge.net/CanFestival-3/ canfestival
RUN cd /src/canfestival && ./configure --prefix=/usr && make  && make install

#Build marble
RUN mkdir -p /src/marblelib/build
RUN cd /src/marblelib/build && cmake -DCMAKE_BUILD_TYPE=Debug -DQTONLY=1 -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local/lib/cmake -DBUILD_WITH_DBUS=0 -DQT_NO_DBUS=1 -DBUILD_MARBLE_APPS=OFF ../
RUN cd /src/marblelib/build && make -j12  && make install

#Build mapwidget
RUN mkdir -p /src/map/build
RUN cd /src/map/build && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local/lib/cmake -DCMAKE_MODULE_PATH=/usr/local/share/marble/cmake ../
RUN cd /src/map/build && make -j12 && make install

RUN useradd -m mother
RUN echo "mother ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

CMD bash
