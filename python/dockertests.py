#!/usr/bin/python
# -*- coding: utf-8 -*-

import subprocess
from dockermanipulator import DockerManipulator
from files import TestFile
prefix='/home/mmacias/r5cop_benchmarking/docker/'


imgs=[]


name='mother:qt'
dockerfile = None
path = prefix+ 'qt/'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})

name='mother:qt_ros'
dockerfile = None
path = prefix+ 'qt_ros'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})

name='mother:nvidia_qt'
dockerfile = None
path = prefix+ 'nvidia_qt/'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})

name='mother:nvidia_qt_ros'
dockerfile = None
path = prefix+ 'qt_ros'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})

name='mother:dep_qt'
dockerfile = None
path = prefix+ 'dep_qt'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})

name='mother:dep_qt_ros'
dockerfile = None
path = prefix+ 'dep_qt_ros'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})


name='mother:dds'
dockerfile = None
path = prefix+ 'dds'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})


name='mother:dep_qt_ros2'
dockerfile = None
path = prefix+ 'dep_qt_ros2'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})

name='mother:mother_rtrc'
dockerfile = None
path = prefix+ 'mother_rtrc'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})
name='mother:mother_ros'
dockerfile = None
path = prefix+ 'mother_ros'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})
name='mother:mother_ros2'
dockerfile = None
path = prefix+ 'mother_ros2'
imgs.append({'Name':name,'Dockerfile':dockerfile, 'Path':path})


c = DockerManipulator(purge=False)
#c.prepareImages(imgs,block=True)
#c.printInfo()

#testlist=['1%','2%','5%','10%'

#cont_robot={'Image':'mother:mother_rtrc','Command':'/tests/robot_rtrc.sh','Hostname':'robot'}
#cont_console={'Image':'mother:mother_rtrc','Command':'/tests/console_rtrc.sh','Hostname':'console'}
#test={'Name':'RTRC 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}



#cont_robot={'Image':'mother:mother_rtrc','Command':'terminator','Hostname':'robot'}
#cont_console={'Image':'mother:mother_rtrc','Command':'terminator','Hostname':'console'}
#test={'Name':'RTRC 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}



#cont_robot={'Image':'mother:mother_ros','Command':'/tests/robot_ros.sh','Hostname':'robot'}
#cont_console={'Image':'mother:mother_ros','Command':'/tests/console_ros.sh','Hostname':'console'}
#test={'Name':'ROS1 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}

#cont_robot={'Image':'mother:mother_ros','Command':'terminator','Hostname':'robot'}
#cont_console={'Image':'mother:mother_ros','Command':'terminator','Hostname':'console'}
#test={'Name':'ROS1 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}


cont_robot={'Image':'mother:mother_ros2','Command':'/tests/robot_ros2.sh','Hostname':'robot'}
cont_console={'Image':'mother:mother_ros2','Command':'/tests/console_ros2.sh','Hostname':'console'}
test={'Name':'ROS2 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}
#test={'Name':'ROS2 0% LOSS','Command':'ls'}

#cont_robot={'Image':'mother:mother_ros2','Command':'terminator','Hostname':'robot'}
#cont_console={'Image':'mother:mother_ros2','Command':'terminator','Hostname':'console'}
#test={'Name':'ROS1 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}


c.test(cont_robot,cont_console,test,tcpdump=False,logs=True)
time=c.getTimeStr()
t=TestFile(time)
t.plotCmdVel()
t.plotRobotInformation()
t.plotSave()
