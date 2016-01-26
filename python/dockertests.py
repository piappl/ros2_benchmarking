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



cont_robot={'Image':'mother:mother_ros','Command':'/tests/robot_ros.sh','Hostname':'robot'}
cont_console={'Image':'mother:mother_ros','Command':'/tests/console_ros.sh','Hostname':'console'}
test={'Name':'ROS1 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}

#cont_robot={'Image':'mother:mother_ros','Command':'terminator','Hostname':'robot'}
#cont_console={'Image':'mother:mother_ros','Command':'terminator','Hostname':'console'}
#test={'Name':'ROS1 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}


#cont_robot={'Image':'mother:mother_ros2','Command':'/tests/robot_ros2.sh','Hostname':'robot'}
#cont_console={'Image':'mother:mother_ros2','Command':'/tests/console_ros2.sh','Hostname':'console'}
#test={'Name':'ROS2 50% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 50%'}
#test={'Name':'ROS2 0% LOSS','Command':'ls'}

#cont_robot={'Image':'mother:mother_ros2','Command':'terminator','Hostname':'robot'}
#cont_console={'Image':'mother:mother_ros2','Command':'terminator','Hostname':'console'}
#test={'Name':'ROS1 10% LOSS','Command':'sudo /sbin/tc qdisc replace dev {} root netem loss 10%'}


containers=[
        [{'Image':'mother:mother_rtrc','Command':'/tests/robot_rtrc.sh','Hostname':'robot'},
            {'Image':'mother:mother_rtrc','Command':'/tests/console_rtrc.sh','Hostname':'console'}],
        [{'Image':'mother:mother_ros','Command':'/tests/robot_ros.sh','Hostname':'robot'},
            {'Image':'mother:mother_ros','Command':'/tests/console_ros.sh','Hostname':'console'}],
        [{'Image':'mother:mother_ros2','Command':'/tests/robot_ros2.sh','Hostname':'robot'},
            {'Image':'mother:mother_ros2','Command':'/tests/console_ros2.sh','Hostname':'console'}]
        ]

def run_loss_tests(tests):
    for cont in containers:
        for test in tests:
            cont_robot=cont[0]
            cont_console=cont[1]
            testname='Image: '+cont[0].get('Image')+' Test: '+ str(test) +' loss'
            testcmd='sudo /sbin/tc qdisc replace dev {} root netem loss '+str(test)+ '%'
            testdict={'Name':testname,'Command':testcmd}
            c.test(cont_robot,cont_console,testdict,tcpdump=True,logs=True,testtime=240)
            time=c.getTimeStr()

def run_delay_tests(tests):
    for cont in containers:
        for test in tests:
            cont_robot=cont[0]
            cont_console=cont[1]
            testname='Image: '+cont[0].get('Image')+' Test: '+ str(test) +' delay'
            testcmd='sudo /sbin/tc qdisc replace dev {} root netem delay '+str(test)+ 'ms'
            testdict={'Name':testname,'Command':testcmd}
            c.test(cont_robot,cont_console,testdict,tcpdump=True,logs=True,testtime=240)
            time=c.getTimeStr()



loss_tests=[0,1,5,10,20,30,40,60,80,90]
delay_tests=[0,10,50,100,500,800,1000]

if __name__ == "__main__":
    run_loss_tests(loss_tests)
    run_delay_tests(delay_tests)

