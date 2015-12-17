FROM mother:dep_qt

USER root

#Get some sources in right places
ADD mother /home/mother/mother

RUN chown -R mother:mother /home/mother/mother

RUN echo '/home/mother/mother/bin' > /etc/ld.so.conf.d/mother.conf
RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/mother.conf
RUN ldconfig

USER mother
ENV HOME /home/mother
ENV LD_LIBRARY_PATH /usr/lib:/home/mother/mother/bin:/usr/local/lib/x86_64-linux-gnu
ENV PATH /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/mother/mother/bin
ENV QT_X11_NO_MITSHM 1
ENV DISPLAY :0


#Build mother
RUN cd /home/mother/mother && /usr/local/bin/qmake -r -spec linux-g++ CONFIG+=debug DEFINES+=TURN_LOGGING_ON INCLUDEPATH+=/usr/include/canfestival CONFIG+=BUILD_PSOR CONFIG+=BUILD_SCOUT 
RUN cd /home/mother/mother && make -j12

CMD /usr/bin/terminator
#CMD bash
