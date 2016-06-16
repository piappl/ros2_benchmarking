import docker, time, os, sys, subprocess, re
from Logs import Logs
from Plotter import Plotter


class TestRunner:
    images = [ "ros1", "ros1node", "ros2", "ros2node", "opensplice", "opensplicenode" ]
    commands = [ "RobotControl", "RobotAlarm", "RobotSensor" ]
    workers = []

    def __init__(self):
        self.docker = docker.Client(base_url='unix://var/run/docker.sock')

    def dirs(self):
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
            print("Forcing container to stop {}: {}".format(container, str(e)), file=sys.stderr)
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
            brctl = subprocess.Popen("brctl show {}".format(bridge), shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
            result = re.search("{}\s+(?P<id>\S+)\s+(?P<stp>\S+)\s+(?P<ifs>.+)".format(bridge), brctl, re.S)
            if result:
                results = re.findall("\S+", result.group("ifs"))
                for interface in results:
                    interfaces.append(interface)
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

    def corruption(self, comm, corruptions, skip):
        logs = Logs()
        plotter = Plotter()
        for corruption in corruptions:
            tid = "{}-corruption-{:02d}".format(comm, corruption)
            tc = "netem corrupt {}%".format(corruption)
            title = "Corruption {}% ({})".format(corruption, comm.upper())
            self.run(comm, tid, tc, title, corruption, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                prefix = "{}-corruption-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), "Lost packets as a function of corrupted packets [%] ({})".format(cmd))
                plotter.firstReceived("{}-first-received".format(prefix), "First packet received as a function of corrupted packets [%] ({})".format(cmd))
                plotter.throughput("{}-throughput".format(prefix), "Throughput [bytes/second] ({})".format(cmd))
                plotter.latency("{}-latency".format(prefix), "Latency [ms] ({})".format(cmd))

    def reorder(self, comm, reorders, skip):
        logs = Logs()
        plotter = Plotter()
        for reorder in reorders:
            tid = "{}-reorder-{:02d}".format(comm, reorder)
            tc = "netem reorder {}% delay 25ms".format(reorder)
            title = "Reorders {}% ({})".format(reorder, comm.upper())
            self.run(comm, tid, tc, title, reorder, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                prefix = "{}-reorder-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), "Lost packets as a function of reordered packets [%] ({})".format(cmd))
                plotter.firstReceived("{}-first-received".format(prefix), "First packet received as a function of reordered packets [%] ({})".format(cmd))
                plotter.throughput("{}-throughput".format(prefix), "Throughput [bytes/second] ({})".format(cmd))
                plotter.latency("{}-latency".format(prefix), "Latency [ms] ({})".format(cmd))

    def duplication(self, comm, dups, skip):
        logs = Logs()
        plotter = Plotter()
        for dup in dups:
            tid = "{}-duplication-{:02d}".format(comm, dup)
            tc = "netem duplicate {}%".format(dup)
            title = "Duplication {}% ({})".format(dup, comm.upper())
            self.run(comm, tid, tc, title, dup, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                prefix = "{}-duplication-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), "Lost packets as a function of duplicated packets [%] ({})".format(cmd))
                plotter.firstReceived("{}-first-received".format(prefix), "First packet received as a function of duplicated packets [%] ({})".format(cmd))
                plotter.throughput("{}-throughput".format(prefix), "Throughput [bytes/second] ({})".format(cmd))
                plotter.latency("{}-latency".format(prefix), "Latency [ms] ({})".format(cmd))

    def limit(self, comm, limits, skip):
        logs = Logs()
        plotter = Plotter()
        for limit in limits:
            tid = "{}-limit-{:04d}kbit".format(comm, limit)
            tc = "tbf rate {}kbit burst 10kbit latency 100ms".format(limit)
            title = "Limit {}kbit ({})".format(limit, comm.upper())
            self.run(comm, tid, tc, title, limit, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                prefix = "{}-limit-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), "Lost packets as a function of throughput impairment [kbit] ({})".format(cmd))
                plotter.firstReceived("{}-first-received".format(prefix), "First packet received as a function of throughput impairment [kbit] ({})".format(cmd))
                plotter.throughput("{}-throughput".format(prefix), "Throughput [bytes/second] ({})".format(cmd))
                plotter.latency("{}-latency".format(prefix), "Latency [ms] ({})".format(cmd))

    def loss(self, comm, losses, skip):
        logs = Logs()
        plotter = Plotter()
        for loss in losses:
            tid = "{}-loss-{:02d}".format(comm, loss)
            tc = "netem loss {}%".format(loss)
            title = "Loss {}% ({})".format(loss, comm.upper())
            self.run(comm, tid, tc, title, loss, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                prefix = "{}-loss-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), "Lost packets as a function of loss impairment [%] ({})".format(cmd))
                plotter.firstReceived("{}-first-received".format(prefix), "First packet received as a function of loss impairment [%] ({})".format(cmd))
                plotter.throughput("{}-throughput".format(prefix), "Throughput [bytes/second] ({})".format(cmd))
                plotter.latency("{}-latency".format(prefix), "Latency [ms] ({})".format(cmd))

    def delay(self, comm, delays, skip):
        logs = Logs()
        plotter = Plotter()
        for delay in delays:
            tid = "{}-delay-{:04d}".format(comm, delay)
            tc = "netem delay {}ms".format(delay)
            title = "Delay {}ms ({})".format(delay, comm.upper())
            self.run(comm, tid, tc, title, delay, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                prefix = "{}-delay-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), "Lost packets as a function of delay impairment [ms] ({})".format(cmd))
                plotter.firstReceived("{}-first-received".format(prefix), "First packet received as a function of delay impairment [ms] ({})".format(cmd))
                plotter.throughput("{}-throughput".format(prefix), "Throughput [bytes/second] ({})".format(cmd))
                plotter.latency("{}-latency".format(prefix), "Latency [ms] ({})".format(cmd))

    def extract(self, logs, prefix, cmd):
        logs.extractLostPackets('data/{}-lost-packets.dat'.format(prefix), cmd)
        logs.extractFirstReceived('data/{}-first-received.dat'.format(prefix), cmd)
        logs.extractThroughput('data/{}-throughput.dat'.format(prefix), cmd)
        logs.extractLatency('data/{}-latency.dat'.format(prefix), cmd)

    def run(self, comm, tid, tc, prefix, value, skip, logs):
        plotter = Plotter()
        if not skip:
            self.execute(comm, tid, tc)
        else:
            print("Using results from: {}".format(tid), file=sys.stderr)
        logs.parse("logs/{}-robot.txt".format(tid), value, "robot")
        logs.parse("logs/{}-console.txt".format(tid), value, "console")
        for cmd in self.commands:
            title = prefix + "[" + cmd + "]";
            logs.extractCommand('data/{}-{}-pub.dat'.format(tid, cmd), value, cmd, "Published")
            logs.extractCommand('data/{}-{}-rec.dat'.format(tid, cmd), value, cmd, "Received")
            plotter.compareTimes(tid, cmd, title)
            plotter.publishedHistogram(tid, cmd, title)
            plotter.receivedHistogram(tid, cmd, title)

    def execute(self, comm, tid, tc):
        print("Running test: {}".format(tid), file=sys.stderr)
        try:
            if comm== "ros1":
                self.ros1(tid, tc)
            elif comm == "ros2":
                self.ros2(tid, tc)
            elif comm == "opensplice":
                self.opensplice(tid, tc)
            os.rename('logs/robot.txt', 'logs/{}-robot.txt'.format(tid))
            os.rename('logs/console.txt', 'logs/{}-console.txt'.format(tid))
        except Exception as e:
            print('Test {} failed: {}'.format(tid, str(e)), file=sys.stderr)
            self.kill()
            self.remove_nodes(comm)

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

