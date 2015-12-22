#!/usr/bin/python
# -*- coding: utf-8 -*-


from files import TestFile
import glob,os,sys
import numpy as np
import matplotlib.pyplot as plt

import matplotlib.mlab as mlab
import copy
from operator import truediv,sub

protocols=['mother:mother_rtrc','mother:mother_ros','mother:mother_ros2']
names={'mother:mother_rtrc':'RTRC','mother:mother_ros':'ROS','mother:mother_ros2':'ROS2'}

def genFileName(test_name,test=None,protocol=None,name=None):
    filename=test_name
    print(test)
    if name:
        filename=filename+'_'+name
    if protocol:
        filename=filename+'_'+protocol
    if test!=None:
        filename=filename+'_'+('%03d'% test)+'_loss'

    print(filename)
    return filename


def genPlotTitle(test_name,test=None,protocol=None,name=None):
    plottitle=''
    if protocol:
        plottitle='Protocol: '+protocol
    if test!=None:
        plottitle=plottitle+' Test: '+('%d'% test)+' loss'
    return plottitle


def plotSingleImage(d,protocol,test,test_name):
    fig=plt.figure()
    fig.suptitle(genPlotTitle(test_name,test,protocol))
    sub=plt.subplot(221)

    scale, data_s = (list(t) for t in zip(*sorted(zip(d["Test"], d["Send"]))))
    plt.plot(scale,data_s,'r-*',label='Send')
    scale, data_r = (list(t) for t in zip(*sorted(zip(d["Test"], d["Received"]))))
    plt.plot(scale,data_r,'b-*',label='Received')
    divided=map(truediv,  data_s,data_r)

    plt.legend()
    plt.xlabel("Link loss [%]")
    plt.ylabel("Packets")

    sub=plt.subplot(222)
    divided, = plt.plot(scale,divided,'g-*',label="Radio")
    plt.xlabel("Link loss [%]")
    plt.ylabel("Ratio")

    sub=plt.subplot(223)
    scale, timemean = (list(t) for t in zip(*sorted(zip(d["Test"], d["TimeMean"]))))
    print(timemean)
    scale, timestd = (list(t) for t in zip(*sorted(zip(d["Test"], d["TimeStd"]))))
    plt.plot(scale,timemean,'y-*',label="Mean")
    plt.xlabel("Link loss [%]")
    plt.ylabel("Mean of reception time derivative [s]")

    sub=plt.subplot(224)
    print(timestd)
    plt.plot(scale,timestd,'g-*',label="Std")
    plt.xlabel("Link loss [%]")
    plt.ylabel("Std of reception time derivative")
    fig.savefig(genFileName(test_name,protocol=protocol)+".png",dpi=300)

def timeHistogram(data,protocol,test,test_name):
    fig=plt.figure()
    plt.suptitle(genPlotTitle(test_name,protocol=protocol,test=test))
    b=np.arange(0,5.1,0.1)
    n, bins, patches = plt.hist(data, bins=b, facecolor='green', alpha=0.75)
    plt.xlim([0,5])
    plt.ylim([0,1000])
    plt.xlabel("Time [s]")
    plt.ylabel("Number of packets")
    fig.savefig(genFileName(test_name,protocol=protocol,test=test,name='histogram')+".png",dpi=300)




protocols=['mother:mother_rtrc','mother:mother_ros','mother:mother_ros2']
names={'mother:mother_rtrc':'RTRC','mother:mother_ros':'ROS','mother:mother_ros2':'ROS2'}

if __name__ == "__main__":
    test_name="first_test_beacons"
    #test_name="second_test_beacons"
    files = glob.glob("./"+test_name+"/summary*.txt")
    datapackets={"mother:mother_rtrc":{"Test":[],"Received":[],"Send":[],"TimeMean":[],"TimeStd":[]},"mother:mother_ros":{"Test":[],"Received":[],"Send":[],"TimeMean":[],"TimeStd":[]},"mother:mother_ros2":{"Test":[],"Received":[],"Send":[],"TimeMean":[],"TimeStd":[]}}
    for f in files:
        fdate=f.split('/')[-1].split('.')[0]
        fdate="-".join(fdate.split('-')[1:3])
        print(fdate)
        t=TestFile(fdate,prefix='/'.join(f.split('/')[0:-1])+'/')
        testname=int(t.getTestName().split(' ')[-2])
        print([testname,t.getStatistics()])
        #t.plotRobotInformation()
        #t.plotSave()
        datapackets[t.getConsoleImage()]['Test'].append(testname)
        datapackets[t.getConsoleImage()]['Received'].append(t.getStatistics()['Received'])
        datapackets[t.getConsoleImage()]['Send'].append(t.getStatistics()['Send'])
        datapackets[t.getConsoleImage()]['TimeMean'].append(t.getTimeDerivative()['TimeMean'])
        datapackets[t.getConsoleImage()]['TimeStd'].append(t.getTimeDerivative()['TimeStd'])

        timeHistogram(t.getTimeDerivatives(),names[t.getConsoleImage()],testname,test_name)
        #print([testname,t.getStatistics()])
        #t.getTimeDerivative()
        #plt.show()
        #sys.exit()
        #sys.exit()
    print(datapackets)
    plotSingleImage(datapackets["mother:mother_rtrc"],'RTRC',None,test_name)
    plotSingleImage(datapackets["mother:mother_ros"],'ROS',None,test_name)
    plotSingleImage(datapackets["mother:mother_ros2"],'ROS2',None,test_name)

#plt.show()
