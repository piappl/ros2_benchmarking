FROM ros:jade
 
RUN apt-get update && apt-get install -y terminator vim build-essential perl python git wget tcpdump
RUN apt-get build-dep -y qt5-default

RUN mkdir /inst && cd /inst && wget \
  http://download.qt.io/official_releases/qt/5.5/5.5.1/single/qt-everywhere-opensource-src-5.5.1.tar.gz
RUN mkdir /src && cd /src && tar -xvf /inst/qt-everywhere-opensource-src-5.5.1.tar.gz
RUN mv /src/qt-everywhere-opensource-src-5.5.1 /src/qt
RUN cd /src/qt && ./configure \
  -confirm-license -opensource \
  -nomake examples -nomake tests -no-compile-examples \
  -prefix "/usr/local"

RUN cd /src/qt && make -j12 all && make install

RUN bash

