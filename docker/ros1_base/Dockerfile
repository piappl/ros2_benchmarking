FROM ros:jade
 
RUN cd /etc && rm localtime && ln -s /usr/share/zoneinfo/Poland localtime
RUN apt-get update && apt-get install -y terminator vim build-essential perl python git wget tcpdump qt5-default qtcreator
