#!/usr/bin/python
# -*- coding: utf-8 -*-


from files import TestFile
import glob,os,sys
import numpy as np
import matplotlib.pyplot as plt
import copy
from operator import truediv,sub
def plotSingleImage(d,title):
    fig=plt.figure()
    fig.suptitle(title)
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




if __name__ == "__main__":
    print("First tests")
    files = glob.glob("./first_test_beacons/summary*.txt")
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

        print([testname,t.getStatistics()])
        t.getTimeDerivative()
        #sys.exit()
    print(datapackets)
    plotSingleImage(datapackets["mother:mother_rtrc"],'RTRC')
    plotSingleImage(datapackets["mother:mother_ros"],'ROS')
    plotSingleImage(datapackets["mother:mother_ros2"],'ROS2')

plt.show()
