#!/usr/bin/python



from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from datetime import timedelta
import math
class TestFile():
    al=u'\u2190'
    at=u'\u2191'
    ar=u'\u2192'
    ad=u'\u2193'
    robot_log=[]
    console_log=[]
    figures=[]
    def __init__(self,ts,prefix='./'):
        self.ts=ts
        self.robot_log=[]
        self.console_log=[]
        self.figures=[]
        self.prefix=prefix
        self.loadAll(ts)


    def loadAll(self,ts):
        summary_name=self.prefix+'summary-{}.txt'.format(ts)
        robot_log_name=self.prefix+'logs-{}-robot.txt'.format(ts)
        console_log_name=self.prefix+'logs-{}-console.txt'.format(ts)
        self.summary=eval(open(summary_name).read())
        print(self.summary)
        print('Test: "'+self.summary.get('Test').get('Name')+'"')
        self.starttime=datetime.strptime(self.summary.get('Teststart'),"%Y%m%d-%H%M%S")-timedelta(hours=1)
        print(str(self.starttime-timedelta(hours=1)))
        print("Robot log {}, console log {}".format(len(self.robot_log),len(self.console_log)))
        f = open(robot_log_name,'r')
        for line in f.readlines():
            if len(line.split())>2 and line.split()[2]=='BENCHMARK!':
                self.robot_log.append(line)
        f.close()
        f = open(console_log_name,'r')
        for line in f.readlines():
            if len(line.split())>2 and line.split()[2]=='BENCHMARK!':
                self.console_log.append(line)
        f.close()

        print("Robot log {}, console log {}".format(len(self.robot_log),len(self.console_log)))
        self.parseRobotInformation()

    def lineToTs(self,line):
        return datetime.strptime(line[0]+' '+line[1],'%Y-%m-%d %H:%M:%S.%f')

    def getData(self,log,direction,command,val):
        pdata=[]
        ptime=[]
        for line in log:
            sl=line.split()
            if sl[3]==direction and sl[4]==command:
                pdata.append(val)
                delta=self.lineToTs(sl)-self.starttime
                #print(val,str(delta),delta.total_seconds(),sl)
                ptime.append(delta.total_seconds())
        return pdata,ptime
    def plotCmdVel(self):
        pdata_r,ptime_r = self.getData(self.robot_log,'RECEIVED','cmd_vel:',2)
        pdata_t,ptime_t = self.getData(self.console_log,'PUBLISHING','cmd_vel:',1)
        self.plotEvents(pdata_t,ptime_t,pdata_r,ptime_r,'cmd_vel, '+self.summary.get('Test').get('Name'))
    def parseRobotInformation(self):
        self.dataRI_DT,self.dataRI_TT = self.getData(self.robot_log,'PUBLISHING','robot_information:',2)
        self.dataRI_DR,self.dataRI_TR = self.getData(self.console_log,'RECEIVED','robot_information:',1)

    def plotRobotInformation(self):
        self.plotEvents(self.dataRI_DT,self.dataRI_TT,self.dataRI_DR,self.dataRI_TR,'robot_information, '+self.summary.get('Test').get('Name'),right=False)

    def getStatistics(self):
        return {"Received":len(self.dataRI_DT),"Send":len(self.dataRI_DR)}
    def getSummary(self):
        return self.summary
    def getConsoleImage(self):
        return self.getSummary().get('Console').get('Image')
    def getRobotImage(self):
        return self.getSummary().get('Robot').get('Image')
    def getTestName(self):
        return self.getSummary().get('Test').get("Name")

    def plotCmdVelAndBeacon(self):
        pdata_r,ptime_r = self.getData(self.robot_log,'RECEIVED','cmd_vel:',2)
        pdata_t,ptime_t = self.getData(self.console_log,'PUBLISHING','cmd_vel:',1)
        bdata_r,btime_r = self.getData(self.console_log,'RECEIVED','robot_information:',1)
        bdata_t,btime_t = self.getData(self.robot_log,'PUBLISHING','robot_information:',2)
        plt.figure()
        plt.plot(pdata_t,ptime_t,'b.',pdata_r,ptime_r,'r.',bdata_t,btime_t,'g.',bdata_r,btime_r,'m.')
        for x0, y0 in zip(pdata_t, ptime_t):
            plt.text(x0, y0, self.ar, size=30, va='center', ha='center', clip_on=True)
        for x0, y0 in zip(pdata_r, ptime_r):
            plt.text(x0, y0, self.ar, size=30, va='center', ha='center', clip_on=True)
        for x0, y0 in zip(bdata_t, btime_t):
            plt.text(x0, y0, self.al, size=30, va='center', ha='center', clip_on=True)
        for x0, y0 in zip(bdata_r, btime_r):
            plt.text(x0, y0, self.al, size=30, va='center', ha='center', clip_on=True)
        plt.xlim([0,3])


    def getTimeDerivative(self):

        dy = np.diff(self.dataRI_TR)
        #print(self.dataRI_TR,dy)
        #print({"TimeMean":np.mean(dy),"TimeStd":np.std(dy)})
        return {"TimeMean":np.mean(dy),"TimeStd":np.std(dy)}
    def getTimeDerivatives(self):
        return np.diff(self.dataRI_TR)
    def plotEvents(self,pdata_t,ptime_t,pdata_r,ptime_r,title="",right=True):
        fig=plt.figure()
        ax = fig.add_subplot(111)
        fig.suptitle(title, fontsize=12)
        self.figures.append(fig)
        plt.ylabel("Time [s]")

        my_xticks = ['Console','Robot']
        plt.xticks([1,2], my_xticks)
        plt.plot(pdata_t,ptime_t,'b.',pdata_r,ptime_r,'r.')
        plt.plot([0, 1, 2, 3],[0,0,0,0],'r-')
        for x0, y0 in zip(pdata_t, ptime_t):
            if right:
                plt.text(x0, y0, self.ar, size=15, va='center', ha='center', clip_on=True)
            else:
                plt.text(x0, y0, self.al, size=15, va='center', ha='center', clip_on=True)
        for x0, y0 in zip(pdata_r, ptime_r):
            if right:
                plt.text(x0, y0, self.ar, size=15, va='center', ha='center', clip_on=True)
            else:
                plt.text(x0, y0, self.al, size=15, va='center', ha='center', clip_on=True)

        plt.xlim([0,3])

    def plotShow(self):
        plt.show()
    def plotSave(self):
        ii=1;
        for fig in self.figures:
            fig.savefig('figure-{}_{}.png'.format(self.ts,str(ii)),dpi=300)
            ii+=1




if __name__ == "__main__":
    #t=TestFile("20151216-143230")
    #t=TestFile("20151216-160652")
    #t=TestFile("20151216-160425")
    #t=TestFile("20151216-163937")
    #t=TestFile("20151216-164103")

    t=TestFile("20151217-104400")
    #t=TestFile("20151217-103033")
    t.plotCmdVel()
    t.plotRobotInformation()
    #t.plotCmdVelAndBeacon()

    #t2=TestFile("20151216-160652")
    #t2.plotCmdVel()
    #t2.plotCmdVelAndBeacon()
    #t.plotShow()
    t.plotSave()
