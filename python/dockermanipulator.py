#!/usr/bin/python
# -*- coding: utf-8 -*-



from __future__ import print_function
from docker import Client
from io import BytesIO
import time, sys, os
from pybrctl import BridgeController
import subprocess
import time


class DockerManipulator():
    """Class for manipulating docker containers for testing robot console communication.
    """
    def __init__(self,base_url='unix://var/run/docker.sock',purge=False,netname='mother'):
        """Constructor of DockeManipulator class. Creates docker-py Client instance,
        setups host_config, volumes, etc.

        Docker netetwork has to be created earlier.

        For test setup this requires host system to have following commands:
          - /sbin/tc
          - /usr/bin/killall
          - /usr/sbin/tcpdump
        And be able to call them with sudo.



        Args:
            base_url (str): Docker controll socket.
            purge (bool): When true, all docker images will be removed.
            netname (string): Network name, passed to --net parameter of docker containers.

        """
        print("Creating docker client with url: "+str(base_url))
        self.cli=Client(base_url=base_url)
        self.netname=netname
        self.host_config=self.cli.create_host_config(devices=[
            '/dev/dri/card0:/dev/dri/card0:rwm'
        ],binds=['/tmp/.X11-unix:/tmp/.X11-unix','/home/mmacias/r5cop_benchmarking/tests/:/tests'],
        privileged=True,network_mode=netname
        )
        self.volumes=['/tmp/.X11-unix','/tests']
        #self.net_id=self.cli.create_network(self.netname,options={'b':''}).get('Id')
        if (purge):
            self.purge()
    def purge(self):
        """Purge removes all containers and images.
        """
        self.removeAllContainers()
        self.removeAllImages()

    def removeAllContainers(self):
        """Remove all containers
        """
        print("Removing containers...")
        conts=self.cli.containers(all=True)
        if len(conts)>0:
            for cont in conts:
                self.cli.stop(cont)
                self.cli.wait(cont)
                self.cli.remove_container(cont)
            print("Done")
        else:
            print("Nothing to remove")
    def removeAllImages(self):
        """Remove all images
        """
        print("Removing images...")
        imgs=self.cli.images()
        if len(imgs)>0:
            for img in imgs:
                self.cli.remove_image(img,force=True)
            print("Done")
        else:
            print("Nothing to remove")
    def prepareImages(self,imgs,block=True):
        '''This method builds images from given specs.

        Args:
            imgs: This is list of dictionaries, with either dockerfile content
                    or file path (when Dockerfile is None).
        '''

        print("Building images...")
        for img in imgs:
            tag = img['Name']
            print("  Tag: "+tag)
            if img['Dockerfile']==None:
                os.chdir(img['Path'])
                if block:
                    #This waits for build to execute
                    #response = [line for line in self.cli.build(fileobj=fil, rm=True, tag=tag)]
                    for line in self.cli.build(path=img['Path'], rm=True, tag=tag):
                        print(line)
                else:
                    #This doesn't wait for build to execute
                    self.cli.build(path=img['Path'],tag=tag,rm=True)
            else:
                fil = BytesIO(img['Path'].encode('utf-8'))
                if block:
                    #This waits for build to execute
                    #response = [line for line in self.cli.build(fileobj=fil, rm=True, tag=tag)]
                    for line in self.cli.build(fileobj=fil, rm=True, tag=tag):
                        print(line)
                else:
                    #This doesn't wait for build to execute
                    self.cli.build(fileobj=fil,tag=tag,rm=True)

            print("  Done")
        print("Done")
    def get_interfaces(self):
        """Returns list with interface used currently on docker network used.

        If no bridge is found, or there is no interfaces, list will be empty.
        """
        br_name='br-'+str(self.cli.networks(names=[self.netname])[0].get('Id')[:12])
        brctl = BridgeController()
        interfaces = []
        try:
            interfaces=brctl.getbr(br_name).getifs()
        except:
            print("Unable to find bridge "+br_name)
        return interfaces
    def getIp(self,container):
        """Returns ip address of given container on docker network

        Args:
            container: Docker container id.
        """
        return self.cli.inspect_container(container).get('NetworkSettings').get('Networks').get(self.netname).get('IPAddress')
    def addHostToHosts(self,container,hostname,ip):
        """Adds given ip address, hostname pair to container /etc/hosts file.
        This is done by executing command on container.

        Args:
            container: Docker container id.
            hostname (str): Hostname to add.
            ip (str): IP address to add.

        """
        cmd='sudo sh -c "echo \'{} {}\' >> /etc/hosts"'.format(ip,hostname)
        print("Running cmd " + cmd + " in container "+container.get('Id'))
        e=self.cli.exec_create(container=container,cmd=cmd)
        self.cli.exec_start(e)
    def getTimeStr(self):
        return self.timestr
    def test(self,cont_robot,cont_console,test,tcpdump=False,logs=False,testtime=0):
        self.timestr = time.strftime("%Y%m%d-%H%M%S")
        print('Starting test "' + test.get('Name') +  '" with timestamp: "'+self.timestr+'"')
        cont_robot_id= self.startContainer(cont_robot)
        cont_console_id= self.startContainer(cont_console)
        cont_robot_ip= self.getIp(cont_robot_id)
        cont_console_ip=self.getIp(cont_console_id)
        print(cont_robot_ip)
        print(cont_console_ip)
        self.addHostToHosts(cont_robot_id,cont_console['Hostname'],cont_console_ip)
        self.addHostToHosts(cont_console_id,cont_robot['Hostname'],cont_robot_ip)
        time.sleep(2)
        if (testtime==0):
            raw_input("Connect to robot and press Enter to start test..")
        else:
            print("Test will start in 5s")
            time.sleep(5)
        timeteststart = time.strftime("%Y%m%d-%H%M%S")
        interfaces=self.get_interfaces()
        print(interfaces)
        pids=[]
        for ii in interfaces:
            cmd=test.get('Command').format(str(ii)).split()
            print(' '.join(cmd))
            subprocess.call(cmd)
            if tcpdump:
              tcpcmd='sudo tcpdump -s 0 -i {} -w {}'.format(str(ii),'data-'+self.timestr+'-'+str(ii)+'.pcap').split()
              print(' '.join(tcpcmd))
              pids.append(subprocess.Popen(tcpcmd).pid)
        time.sleep(1)
        if (testtime==0):
            raw_input("Running test press Enter to finish...")
        else:
            print("Test is running for commanded "+str(testtime)+"s")
            time.sleep(testtime)
        if tcpdump:
            #FIXME this is ugly
            cmd='sudo killall tcpdump'
            print(cmd)
            subprocess.call(cmd.split())
        if logs:
            f = open('logs-'+self.timestr+'-robot.txt','w')
            f.write(self.cli.logs(cont_robot_id))
            f.close()
            f = open('logs-'+self.timestr+'-console.txt','w')
            f.write(self.cli.logs(cont_console_id))
            f.close()
            f = open('summary-'+self.timestr+'.txt','w')
            summary={'Robot':cont_robot,'Console':cont_console,'Test':test,'Timestring':self.timestr,'Interfaces':interfaces,'Teststart':timeteststart}
            f.write(str(summary))
            f.close()
        self.cli.stop(cont_robot_id,timeout=0.1)
        self.cli.stop(cont_console_id,timeout=0.1)
        self.cli.remove_container(cont_robot_id,force=True)
        self.cli.remove_container(cont_console_id,force=True)


    def createContainer(self,cont):
        image=cont['Image']
        command=cont['Command']
        try:
            hostname=cont['Hostname']
        except:
            hostname=None
        container=None
        if hostname:
            print("Creating container with hostname "+hostname)
            container = self.cli.create_container(image=image,command=command,volumes=self.volumes,host_config=self.host_config,hostname=hostname )
        else:
            print("Creating container")
            container  = self.cli.create_container(image=image,command=command,volumes=self.volumes,host_config=self.host_config)
        if container!=None:
            print('Created, id ' + str(container.get('Id')))
        else:
            print('Container not created')
        return container

    def startContainer(self,cont):
        """Creates and starts container with given container description.
        Args:
            cont: Container description is dictionary with fields:
                "Image": Image name.
                "Command": Command to run in image.
                "Hostname": Hostname of created container.
        Example:
          d=DockerManipulator()
          d.startContainer({'Image':'mother:mother_rtrc','Command':'terminator','Hostname':'console'})

        """
        container = self.createContainer(cont)
        self.cli.start(container.get('Id'))
        return container

    def printInfo(self):
        """Print info on container and images.
        """
        print("Containers:")
        print(self.cli.containers(all=True))
        print("Images:")
        print(self.cli.images())


if __name__ == "__main__":
    images=[]
    name='mmacias:test'
    dockerfile = '''
    FROM ros:jade
    CMD ["/bin/sleep","30"]
    '''
    images.append({'Name':name,'Dockerfile':dockerfile})

    name='mmacias:test2'
    dockerfile = '''
    FROM ros:jade
    CMD ["/bin/sleep","30"]
    '''
    images.append({'Name':name,'Dockerfile':dockerfile})

    print(images)


    c = DockerManipulator(purge=False)
    c.prepareImages(images,block=True)
    c.printInfo()
    sys.exit()


