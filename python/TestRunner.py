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
        subprocess.call("rm current".format(directory), shell = True)
        subprocess.call("ln {} current -s".format(directory), shell = True)
        for subdir in ["logs", "data", "graphs", "tcpdump"]:
            subprocess.call("mkdir -p {}/{}".format(directory, subdir), shell = True)
        for subdir in ["times", "lost-packets", "histograms", "first-received", "throughput", "latency"]:
            subprocess.call("mkdir -p {}/graphs/{}".format(directory, subdir), shell = True)


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
            print("Forcing container to stop {}: {}".format(container, str(e)), flush=True, file=sys.stderr)
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
                xlabel = "Corruption rate [%]"
                prefix = "{}-corruption-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                plotter.latency("{}-latency".format(prefix), cmd, xlabel)

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
                xlabel = "Reordering rate [%]"
                prefix = "{}-reorder-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                plotter.latency("{}-latency".format(prefix), cmd, xlabel)

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
                xlabel = "Duplication rate [%]"
                prefix = "{}-duplication-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def limit(self, comm, limits, skip):
        logs = Logs()
        plotter = Plotter()
        for limit in limits:
            tid = "{}-limit-{:04d}kbit".format(comm, limit)
            tc = "tbf rate {}kbit burst 32kbit latency 500ms".format(limit)
            title = "Limit {}kbit ({})".format(limit, comm.upper())
            self.run(comm, tid, tc, title, limit, skip, logs)
        if len(logs.keys()) > 1:
            for cmd in self.commands:
                xlabel = "Throughput limit [kbit]"
                prefix = "{}-limit-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                plotter.latency("{}-latency".format(prefix), cmd, xlabel)

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
                xlabel = "Packet loss rate [%]"
                prefix = "{}-loss-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                plotter.latency("{}-latency".format(prefix), cmd, xlabel)

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
                xlabel = "Packet delay [ms]"
                prefix = "{}-delay-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                plotter.latency("{}-latency".format(prefix), cmd, xlabel)

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
            print("Using results from: {}".format(tid), flush=True)
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
        print("Running test: {}".format(tid), flush=True)
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
            print('Test {} failed: {}'.format(tid, str(e)), flush=True, file=sys.stderr)
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

