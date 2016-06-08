import docker, time, os, sys, pybrctl, subprocess
from Logs import Logs
from Plotter import Plotter


class TestRunner:
    images = [ "ros1", "ros1node", "ros2", "ros2node", "opensplice", "opensplicenode" ]
    commands = [ "cmd_vel", "robot_control", "robot_status", "byte_msg" ]
    workers = []

    def __init__(self):
        self.docker = docker.Client(base_url='unix://var/run/docker.sock')
        directory = "results/{}".format(int(time.time()))
        for subdir in ["logs", "data", "graphs", "tcpdump"]:
            subprocess.call("rm -f {}".format(subdir), shell = True)
            subprocess.call("mkdir -p {}/{}".format(directory, subdir), shell = True)
            subprocess.call("ln -s {}/{} {}".format(directory, subdir, subdir), shell = True)

    def remove_containers(self, name):
        containers = self.docker.containers(filters = { 'ancestor': 'test:{}'.format(name) })
        for container in containers:
            self.docker.stop(container)
            self.docker.wait(container)
            self.docker.remove_container(container)

    def remove_nodes(self, name):
        self.remove_containers(name + "node")

    def wait(self, container, timeout = 360):
        try:
            self.docker.wait(container, timeout)
        except Exception as e:
            print("Forcing container to stop {}: {}".format(container, str(e)))
            self.docker.stop(container)

    def kill(self):
        for proc in self.workers:
            if proc:
                proc.kill()
        self.workers = []

    def interfaces(self, network, expected):
        interfaces = []
        try:
            bridge = 'br-' + str(self.docker.networks(names=[network])[0].get('Id')[:12])
            brctl = pybrctl.BridgeController()
            interfaces = brctl.getbr(bridge).getifs()
        except Exception as e:
            raise RuntimeError("Unable to find bridge for network {}: {}".format(network, str(e)))
        if len(interfaces) != expected:
            raise RuntimeError("Expected {} interface(s) for network {}".format(expected, network))
        return interfaces

    def tcpdump(self, interface, filename):
        devnull = open(os.devnull, 'w')
        try:
            self.workers.append(subprocess.Popen(['/usr/sbin/tcpdump', '-s', '0', '-i', interface, '-w', 'tcpdump/{}-{}.pcap'.format(filename, interface)], stderr=devnull))
        except:
            raise RuntimeError('Failed to start tcpdump at interface {}: {}'.format(interface, sys.exc_info()[0]))

    def tc(self, interface, param):
        try:
            subprocess.call('/sbin/tc qdisc replace dev {} root {}'.format(interface, param), shell = True)
        except:
            raise RuntimeError('Failed to set "{}" at interface {}: {}'.format(param, interface, sys.exc_info()[0]))

    def plotLostPackets(self, tid, prefix, logs):
        if len(logs) > 1:
            log = Logs()
            plotter = Plotter()
            for cmd in self.commands:
                title = prefix + "[" + cmd + "]";
                log.extractLostPackets('data/{}-{}-lost-packets.dat'.format(tid, cmd), cmd, logs)
                plotter.lostPackets(tid, cmd, title)

    def plotLostPacketsEstablished(self, tid, title, logs):
        if len(logs) > 1:
            log = Logs()
            plotter = Plotter()
            for cmd in self.commands:
                log.extractLostPacketsEstablished('data/{}-lost-packets-established.dat'.format(tid), logs)
                plotter.lostPacketsEstablished(tid, title)

    def plotFirstReceived(self, tid, prefix, logs):
        if len(logs) > 1:
            log = Logs()
            plotter = Plotter()
            for cmd in self.commands:
                title = prefix + "[" + cmd + "]";
                log.extractFirstReceived('data/{}-{}-first-received.dat'.format(tid, cmd), cmd, logs)
                plotter.firstReceived(tid, cmd, title)

    def corruption(self, comm, corruptions):
        logs = []
        for corruption in corruptions:
            tid = "{}-corruption-{:02d}".format(comm, corruption)
            tc = "netem corrupt {}%".format(corruption)
            title = "Corruption {}% ({})".format(corruption, comm.upper())
            logs += self.run(comm, tid, title, corruption, tc)
        self.plotLostPackets("{}-corruption".format(comm), "Lost packets as a function of corrupted packets [%]", logs)
        self.plotFirstReceived("{}-corruption".format(comm), "First packet received as a function of corrupted packets [%]", logs)
        #self.plotLostPacketsEstablished("{}-corruption".format(comm), "Lost packets after establishing a connection as a function of corrupted packages [%]", logs)

    def reorder(self, comm, reorders):
        logs = []
        for reorder in reorders:
            tid = "{}-reorder-{:02d}".format(comm, reorder)
            tc = "netem reorder {}% delay 25ms".format(reorder)
            title = "Reorders {}% ({})".format(reorder, comm.upper())
            logs += self.run(comm, tid, title, reorder, tc)
        self.plotLostPackets("{}-reorder".format(comm), "Lost packets as a function of reordered packets [%]", logs)
        self.plotFirstReceived("{}-reorder".format(comm), "First packet received as a function of reordered packets [%]", logs)
        #self.plotLostPacketsEstablished("{}-reorder".format(comm), "Lost packets after establishing a connection as a function of reordered packages [%]", logs)

    def duplication(self, comm, dups):
        logs = []
        for dup in dups:
            tid = "{}-duplication-{:02d}".format(comm, dup)
            tc = "netem duplicate {}%".format(dup)
            title = "Duplication {}% ({})".format(dup, comm.upper())
            logs += self.run(comm, tid, title, dup, tc)
        self.plotLostPackets("{}-duplication".format(comm), "Lost packets as a function of duplicated packets [%]", logs)
        self.plotFirstReceived("{}-duplication".format(comm), "First packet received as a function of duplicated packets [%]", logs)
        #self.plotLostPacketsEstablished("{}-duplication".format(comm), "Lost packets after establishing a connection as a function of duplicated packages [%]", logs)

    def limit(self, comm, limits):
        logs = []
        for limit in limits:
            tid = "{}-limit-{:04d}kbit".format(comm, limit)
            tc = "tbf rate {}kbit burst 10kbit latency 100ms".format(limit)
            title = "Limit {}kbit ({})".format(limit, comm.upper())
            logs += self.run(comm, tid, title, limit, tc)
        self.plotLostPackets("{}-limit".format(comm), "Lost packets as a function of throughput impairment [kbit]", logs)
        self.plotFirstReceived("{}-limit".format(comm), "First packet received as a function of throughput impairment [kbit]", logs)
        #self.plotLostPacketsEstablished("{}-limit".format(comm), "Lost packets after establishing a connection as a function of throughput impairment [kbit]", logs)

    def loss(self, comm, losses):
        logs = []
        for loss in losses:
            tid = "{}-loss-{:02d}".format(comm, loss)
            tc = "netem loss {}%".format(loss)
            title = "Loss {}% ({})".format(loss, comm.upper())
            logs += self.run(comm, tid, title, loss, tc)
        self.plotLostPackets("{}-loss".format(comm), "Lost packets as a function of loss impairment [%]", logs)
        self.plotFirstReceived("{}-loss".format(comm), "First packet received as a function of loss impairment [%]", logs)
        #self.plotLostPacketsEstablished("{}-loss".format(comm), "Lost packets after establishing a connection as a function of loss impairment [%]", logs)

    def delay(self, comm, delays):
        logs = []
        for delay in delays:
            tid = "{}-delay-{:04d}".format(comm, delay)
            tc = "netem delay {}ms".format(delay)
            title = "Delay {}ms ({})".format(delay, comm.upper())
            logs += self.run(comm, tid, title, delay, tc)
        self.plotLostPackets("{}-delay".format(comm), "Lost packets as a function of delay impairment [ms]", logs)
        self.plotFirstReceived("{}-delay".format(comm), "First packet received as a function of delay impairment [%]", logs)
        #self.plotLostPacketsEstablished("{}-delay".format(comm), "Lost packets after establishing a connection as a function of delay impairment [%]", logs)

    def run(self, comm, tid, prefix, value, tc):
        print("Running test: {}".format(tid))
        try:
            log = Logs()
            plotter = Plotter()
            if comm== "ros1":
                self.ros1(tid, tc)
            elif comm == "ros2":
                self.ros2(tid, tc)
            elif comm == "opensplice":
                self.opensplice(tid, tc)
            os.rename('logs/robot.txt', 'logs/{}-robot.txt'.format(tid))
            os.rename('logs/console.txt', 'logs/{}-console.txt'.format(tid))
            robot = log.parse("logs/{}-robot.txt".format(tid))
            console = log.parse("logs/{}-console.txt".format(tid))
            for cmd in self.commands:
                title = prefix + "[" + cmd + "]";
                log.extractCommand('data/{}-{}-pub.dat'.format(tid, cmd), cmd, "Published", console + robot)
                log.extractCommand('data/{}-{}-rec.dat'.format(tid, cmd), cmd, "Received", console + robot)
                plotter.compareTimes(tid, cmd, title)
                plotter.publishedHistogram(tid, cmd, title)
                plotter.receivedHistogram(tid, cmd, title)
            return [ { 'Console': console, 'Robot': robot, 'Value': value } ]
        except Exception as e:
            print('Test {} failed: {}'.format(tid, str(e)))
            self.kill()
            self.remove_nodes(comm)
            return []

    def ros1(self, tid, tc):
        master_id = subprocess.Popen("./scripts/start_ros1_master.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        master_if = self.interfaces("ros1", 1)[0]
        robot_id = subprocess.Popen("./scripts/start_ros1_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros1_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        interfaces = self.interfaces("ros1", 3)
        for interface in interfaces:
            if interface != master_if:
                self.tc(interface, tc)
                self.tcpdump(interface, tid)
        self.wait(console_id)
        self.wait(robot_id)
        self.docker.stop(master_id)
        self.kill()
        self.docker.remove_container(console_id)
        self.docker.remove_container(robot_id)
        self.docker.remove_container(master_id)

    def ros2(self, tid, tc):
        robot_id = subprocess.Popen("./scripts/start_ros2_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros2_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        interfaces = self.interfaces("ros2", 2)
        for interface in interfaces:
            self.tc(interface, tc)
            self.tcpdump(interface, tid)
        self.wait(console_id)
        self.wait(robot_id)
        self.kill()
        self.docker.remove_container(console_id)
        self.docker.remove_container(robot_id)

    def opensplice(self, tid, tc):
        robot_id = subprocess.Popen("./scripts/start_opensplice_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_opensplice_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        interfaces = self.interfaces("opensplice", 2)
        for interface in interfaces:
            self.tc(interface, tc)
            self.tcpdump(interface, tid)
        self.wait(console_id)
        self.wait(robot_id)
        self.kill()
        self.docker.remove_container(console_id)
        self.docker.remove_container(robot_id)

