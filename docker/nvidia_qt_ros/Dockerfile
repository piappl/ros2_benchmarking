FROM mother:qt_ros
RUN apt-get install -y x-window-system binutils mesa-utils module-init-tools

ADD nvidia-driver.run /tmp/nvidia-driver.run
RUN sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module
RUN rm /tmp/nvidia-driver.run

CMD /bin/bash
