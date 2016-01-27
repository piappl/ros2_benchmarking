#!/usr/bin/python



from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from datetime import datetime
from datetime import timedelta
from operator import truediv,sub
import glob,os,sys,copy,math


class TestFile():
    """Handles things connected to single test case (multiple files actually, but with single timestamp).
    """
    names={'mother:mother_rtrc':'RTRC','mother:mother_ros':'ROS','mother:mother_ros2':'ROS2'}


    al=u'\u2190'
    at=u'\u2191'
    ar=u'\u2192'
    ad=u'\u2193'
    robot_log=[]
    console_log=[]
    figures=[]
    plot_name=None
    def __init__(self,ts,prefix='./',debug=False,plot_name=None,test_name=None):
        self.ts=ts
        self.robot_log=[]
        self.console_log=[]
        self.figures=[]
        self.prefix=prefix
        self.test_name=test_name
        self.plot_name=plot_name
        self.loadAll(ts,debug)


    def loadAll(self,ts,debug=False):
        """Loads all importand datafiles:
            - console log
            - robot log
            - summary file
        Than runs other functions to sanitize data.
        """
        summary_name=self.prefix+'summary-{}.txt'.format(ts)
        robot_log_name=self.prefix+'logs-{}-robot.txt'.format(ts)
        console_log_name=self.prefix+'logs-{}-console.txt'.format(ts)

        self.summary=eval(open(summary_name).read())
        if debug:
          print(self.summary)
        self.starttime=datetime.strptime(self.summary.get('Teststart'),"%Y%m%d-%H%M%S")-timedelta(hours=1)
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
        self.parseRobotInformation()
        self.plotRobotInformation()
        self.cleanupRobotInformation()

    def cleanupRobotInformation(self):
        """This function searchess data for first and last message in robot
        and console, and removes everything before or after.

        This is to remove from tests situation when robot start to send messages
        before console is even started.
        Messages received:
        self.dataRI_MR=[['2016-01-25', '10:41:21.665900', 'BENCHMARK!', 'PUBLISHING', 'robot_information:', 'robot_id=2548,', 'description=UEAEZSEEEZTU']]
        Messages transmitted:
        self.dataRI_MT=[['2016-01-25', '10:41:21.667014', 'BENCHMARK!', 'RECEIVED', 'robot_information:', 'robot_id=2548,', 'description=EUSMAMMTATMS']]
        In result self.dataRI is populated, with paired messages (transmitted/received pair), also
        Data in self.dataRI_* is cropped.
        """
        #This looks for transmit/receive pairs, by robot information description field.
        self.dataRI=[]
        for msg_r in self.dataRI_MR:
            for msg_t in self.dataRI_MT:
                if (msg_r[6]==msg_t[6]):
                    self.dataRI.append({'transmit_ts':self.timeFromStart(msg_t),'receive_ts':self.timeFromStart(msg_r),'content':msg_r[6]})
                    #print({'transmit_ts':self.timeFromStart(msg_t),'receive_ts':self.timeFromStart(msg_r),'content':msg_r[6]})
        #This will remove all messages that are transmitted earlier than first message received, or transmitted later
        #than last message received. This is to cleanup data.
        #Option one
        #self.first_ts=self.dataRI[0]['transmit_ts']
        #self.last_ts=self.dataRI[-1]['transmit_ts']
        #first_ts_index=self.dataRI_TT.index(self.first_ts)
        #last_ts_index=self.dataRI_TT.index(self.last_ts)
        self.dataRI_TT=[n for n in self.dataRI_TT if n>=0 and n<=240]
        self.dataRI_TR=[n for n in self.dataRI_TR if n>=0 and n<=240]
        self.dataRI_DR=self.dataRI_DR[:len(self.dataRI_TR)]
        self.dataRI_DT=self.dataRI_DT[:len(self.dataRI_TT)]
        #print(self.ts)
        #print("Before clean up {} {} {} {} {} {} {}".format(len(self.dataRI_TR),len(self.dataRI_DR),len(self.dataRI_MR),len(self.dataRI_TT),len(self.dataRI_DT),len(self.dataRI_MT),len(self.dataRI)))
        #print("First {} {} last {} {}".format(self.first_ts,self.first_ts_index,last_ts,last_ts_index))
        #self.dataCRI_TT=self.dataRI_TT[first_ts_index:(last_ts_index+1)]
        #self.dataCRI_MT=self.dataRI_MT[first_ts_index:(last_ts_index+1)]
        #self.dataCRI_DT=self.dataRI_DT[first_ts_index:(last_ts_index+1)]
        #print("After  clean up {} {} {} {} {} {} {}".format(len(self.dataRI_TR),len(self.dataRI_DR),len(self.dataRI_MR),len(self.dataRI_TT),len(self.dataRI_DT),len(self.dataRI_MT),len(self.dataRI)))

    def timeFromStart(self,line):
        """Returns time from start for give line, where line
        is list as stored in self.dataRI_* tables.
        """
        return (self.lineToTs(line)-self.starttime).total_seconds()
    def lineToTs(self,line):
        """Returns timestamp from line. Where line is list as
        stored in self.dataRI_* tables.
        """
        return datetime.strptime(line[0]+' '+line[1],'%Y-%m-%d %H:%M:%S.%f')
    def getData(self,log,direction,command,val):
        pdirection=[]
        pmessage=[]
        ptime=[]
        for line in log:
            sl=line.split()
            if sl[3]==direction and sl[4]==command:
                pdirection.append(val)
                pmessage.append(sl)
                ptime.append(self.timeFromStart(sl))
        return pdirection,ptime,pmessage
    def plotCmdVel(self):
        pdirection_r,ptime_r,pmessage_r = self.getData(self.robot_log,'RECEIVED','cmd_vel:',2)
        pdirection_t,ptime_t,pmessage_t = self.getData(self.console_log,'PUBLISHING','cmd_vel:',1)
        self.plotEvents(pdirection_t,ptime_t,pdirection_r,ptime_r,'cmd_vel, '+self.summary.get('Test').get('Name'))
    def parseRobotInformation(self):
        self.dataRI_DT,self.dataRI_TT,self.dataRI_MT = self.getData(self.robot_log,'PUBLISHING','robot_information:',2)
        self.dataRI_DR,self.dataRI_TR,self.dataRI_MR = self.getData(self.console_log,'RECEIVED','robot_information:',1)

    def plotRobotInformation(self):
        if self.plot_name:
            title=self.plot_name+', Protocol: '+self.names[self.getConsoleImage()]+', Test: ' +(self.getTestName().split(' ')[-2])
        else:
            title='Protocol: '+self.names[self.getConsoleImage()]+', Test: ' +(self.getTestName().split(' ')[-2])
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
        pdata_r,ptime_r,a = self.getData(self.robot_log,'RECEIVED','cmd_vel:',2)
        pdata_t,ptime_t,a = self.getData(self.console_log,'PUBLISHING','cmd_vel:',1)
        bdata_r,btime_r,a = self.getData(self.console_log,'RECEIVED','robot_information:',1)
        bdata_t,btime_t,a = self.getData(self.robot_log,'PUBLISHING','robot_information:',2)
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
        dy = self.getTimeDerivatives()
        return {"TimeMean":np.mean(dy),"TimeStd":np.std(dy)}
    def getTimeDerivatives(self):
        drv=[]
        for d in self.dataRI:
            drv.append(d['receive_ts']-d['transmit_ts'])
        return drv
    def plotEvents(self,pdata_t,ptime_t,pdata_r,ptime_r,title="",right=True,arrows=False):
        fig=plt.figure()
        ax = fig.add_subplot(111)
        fig.suptitle(title, fontsize=12)
        self.figures.append(fig)
        plt.ylabel("Time [s]")

        my_xticks = ['Console','Robot']
        plt.xticks([1,2], my_xticks)
        plt.plot(pdata_t,ptime_t,'bo',pdata_r,ptime_r,'ro',markersize=0.5)
        plt.plot([0, 1, 2, 3],[0,0,0,0],'r-')
        if arrows:
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
            if self.test_name:
                title=self.test_name+'_events_' +self.names[self.getConsoleImage()]+'_' +('%05d'% int(self.getTestName().split(' ')[-2]))+".png"
            else:
                title='figure-{}_{}.png'.format(self.ts,str(ii))
            fig.savefig(title,dpi=300)
            fig.clf()
            ii+=1

class TestFiles():
    protocols=['mother:mother_rtrc','mother:mother_ros','mother:mother_ros2']
    names={'mother:mother_rtrc':'RTRC','mother:mother_ros':'ROS','mother:mother_ros2':'ROS2'}


    test_name=''
    plot_name=''
    def __init__(self,test_name,plot_name):
        """ Test name is name of directory with test file. Plot name is human readable name to put on plots.
        Filenames of saved figures will use test_name.
        """
        self.test_name=test_name
        self.plot_name=plot_name
        files = glob.glob("./"+test_name+"/summary*.txt")
        datapackets={"mother:mother_rtrc":{"Test":[],"Received":[],"Send":[],"TimeMean":[],"TimeStd":[]},"mother:mother_ros":{"Test":[],"Received":[],"Send":[],"TimeMean":[],"TimeStd":[]},"mother:mother_ros2":{"Test":[],"Received":[],"Send":[],"TimeMean":[],"TimeStd":[]}}
        for f in files:
            fdate=f.split('/')[-1].split('.')[0]
            fdate="-".join(fdate.split('-')[1:3])
            t=TestFile(fdate,prefix='/'.join(f.split('/')[0:-1])+'/',test_name=self.test_name,plot_name=self.plot_name)
            testname=int(t.getTestName().split(' ')[-2])
            #t.plotRobotInformation()
            #t.plotRobotInformation()
            t.plotSave()
            datapackets[t.getConsoleImage()]['Test'].append(testname)
            datapackets[t.getConsoleImage()]['Received'].append(t.getStatistics()['Received'])
            datapackets[t.getConsoleImage()]['Send'].append(t.getStatistics()['Send'])
            datapackets[t.getConsoleImage()]['TimeMean'].append(t.getTimeDerivative()['TimeMean'])
            datapackets[t.getConsoleImage()]['TimeStd'].append(t.getTimeDerivative()['TimeStd'])
            self.timeHistogram(t.getTimeDerivatives(),self.names[t.getConsoleImage()],testname,self.test_name,self.plot_name)
        self.plotSingleImage(datapackets["mother:mother_rtrc"],'RTRC',None,self.test_name,self.plot_name)
        self.plotSingleImage(datapackets["mother:mother_ros"],'ROS',None,self.test_name,self.plot_name)
        self.plotSingleImage(datapackets["mother:mother_ros2"],'ROS2',None,self.test_name,self.plot_name)
        self.plotCompareProtocols(datapackets,self.protocols,None,self.test_name,self.plot_name)


    def genFileName(self,test_name,test=None,protocol=None,name=None):
        """Note: inputs here are parameters, not class fields on purpose"""
        filename=test_name
        if name:
            filename=filename+'_'+name
        if protocol:
            filename=filename+'_'+protocol
        if test!=None:
            filename=filename+'_'+('%05d'% test)+''
        print(filename)
        return filename


    def genPlotTitle(self,test_name,test=None,protocol=None,name=None):
        """Note: inputs here are parameters, not class fields on purpose"""
        plottitle=test_name
        if protocol:
            if plottitle=='':
                plottitle='Protocol: '+protocol
            else:
                plottitle+=', Protocol: '+protocol
        if test!=None:
            plottitle+=', Test: '+('%d'% test)+''
        return plottitle

    def plotCompareProtocols(self,data,protocols,test,test_name,plot_name):
        fig=plt.figure(figsize=(15,5))
        fig.suptitle(self.genPlotTitle(plot_name,test))
        sub=plt.subplot(131)

        for protocol in protocols:
            d=data[protocol]
            scale, data_s = (list(t) for t in zip(*sorted(zip(d["Test"], d["Send"]))))
            scale, data_r = (list(t) for t in zip(*sorted(zip(d["Test"], d["Received"]))))
            divided=map(truediv,  data_s,data_r)
            divided, = plt.plot(scale,divided,'-*',label=self.names[protocol])

        plt.legend()
        plt.xlabel("Link loss [%]")
        plt.ylabel("Ratio")


        sub=plt.subplot(132)
        for protocol in protocols:
            d=data[protocol]
            scale, timemean = (list(t) for t in zip(*sorted(zip(d["Test"], d["TimeMean"]))))
            plt.plot(scale,timemean,'-*',label=self.names[protocol])
        plt.legend()

        plt.xlabel("Link loss [%]")
        plt.ylabel("Mean delivery time [s]")

        sub=plt.subplot(133)
        for protocol in protocols:
            d=data[protocol]
            scale, timestd = (list(t) for t in zip(*sorted(zip(d["Test"], d["TimeStd"]))))
            plt.plot(scale,timestd,'-*',label=self.names[protocol])
        plt.legend()
        plt.xlabel("Link loss [%]")
        plt.ylabel("Standard deviation of delivery time")

        fig.tight_layout()
        plt.subplots_adjust(top=0.85)
        fig.savefig(self.genFileName(test_name)+".png",dpi=300)




    def plotSingleImage(self,d,protocol,test,test_name,plot_name):
        fig=plt.figure(figsize=(15,10))
        fig.suptitle(self.genPlotTitle(plot_name,test,protocol))
        sub=plt.subplot(221)

        scale, data_s = (list(t) for t in zip(*sorted(zip(d["Test"], d["Send"]))))
        plt.plot(scale,data_s,'r-*',label='Received')
        scale, data_r = (list(t) for t in zip(*sorted(zip(d["Test"], d["Received"]))))
        plt.plot(scale,data_r,'b-*',label='Send')
        print(data_s)
        print(data_r)
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
        scale, timestd = (list(t) for t in zip(*sorted(zip(d["Test"], d["TimeStd"]))))
        plt.plot(scale,timemean,'y-*',label="Mean")
        plt.xlabel("Link loss [%]")
        plt.ylabel("Mean delivery time [s]")

        sub=plt.subplot(224)
        plt.plot(scale,timestd,'g-*',label="Std")
        plt.xlabel("Link loss [%]")
        plt.ylabel("Standard deviation of delivery time")

        fig.tight_layout()
        plt.subplots_adjust(top=0.85)
        fig.savefig(self.genFileName(test_name,protocol=protocol)+".png",dpi=300)

    def timeHistogram(self,data,protocol,test,test_name,plot_name):
        fig=plt.figure()
        plt.suptitle(self.genPlotTitle(plot_name,protocol=protocol,test=test))
        b=np.arange(0,5.1,0.1)
        n, bins, patches = plt.hist(data, bins=b, facecolor='green', alpha=0.75)
        plt.xlim([0,5])
        plt.ylim([0,1000])
        plt.xlabel("Time [s]")
        plt.ylabel("Number of packets")
        fig.savefig(self.genFileName(test_name,protocol=protocol,test=test,name='histogram')+".png",dpi=300)





if __name__ == "__main__":
    #t=TestFile("20151216-143230")
    #t=TestFile("20151216-160652")
    #t=TestFile("20151216-160425")
    #t=TestFile("20151216-163937")
    #t=TestFile("20151216-164103")

    t=TestFile("20160125-113712",prefix="./example/",debug=True)
    #t=TestFile("20151217-103033")
    t.plotCmdVel()
    t.plotRobotInformation()
    #t.plotCmdVelAndBeacon()

    #t2=TestFile("20151216-160652")
    #t2.plotCmdVel()
    #t2.plotCmdVelAndBeacon()
    #t.plotShow()
    t.plotSave()
