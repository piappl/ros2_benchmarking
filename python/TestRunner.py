import docker, time, os, sys, subprocess, re, glob
from Logs import Logs
from Plotter import Plotter


class TestRunner:
    images = [ "ros1:base", "ros1:node", "ros2:base", "ros2:opensplice", "ros2:fastrtps", "ros2:connext", "opensplice:base", "opensplice:node", "ros2:bridge" ]
    tests = [ 'ros1', 'ros2opensplice', 'ros2fastrtps', 'ros2connext', 'opensplice', 'ros1bridge' ]
    commands = [ "RobotControl", "RobotAlarm", "RobotSensor" ]
    workers = []

    def __init__(self):
        self.docker = docker.Client(base_url='unix://var/run/docker.sock')

    def dirs(self):
        directory = "results/{}".format(int(time.time()))
        subprocess.call("rm -f current", shell = True)
        subprocess.call("ln {} current -s".format(directory), shell = True)
        for subdir in ["logs", "data", "graphs", "tcpdump"]:
            subprocess.call("mkdir -p {}/{}".format(directory, subdir), shell = True)
        for subdir in ["times", "lost-packets", "histograms", "first-received", "throughput", "latency"]:
            subprocess.call("mkdir -p {}/graphs/{}".format(directory, subdir), shell = True)


    def remove_containers(self, name):
        containers = self.docker.containers(filters = { 'ancestor': '{}'.format(name) })
        for container in containers:
            self.docker.stop(container)
            self.wait(container, 10)
            self.docker.remove_container(container)

    def kill_stop_remove(self, containers):
        self.kill()
        for container in containers:
            self.docker.stop(container)
            self.docker.remove_container(container)

    def wait_kill_remove(self, containers, cooldown = 0):
        for container in containers:
            self.wait(container)
        time.sleep(cooldown)
        self.kill()
        for container in containers:
            self.docker.remove_container(container)

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
            bridge = 'br-' + str(self.docker.networks(names=["^{}$".format(network)])[0].get('Id')[:12])
            brctl = subprocess.Popen("brctl show {}".format(bridge), shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
            result = re.search("{}\s+(?P<id>\S+)\s+(?P<stp>\S+)\s+(?P<ifs>.+)".format(bridge), brctl, re.S)
            if result:
                results = re.findall("\S+", result.group("ifs"))
                for interface in results:
                    interfaces.append(interface)
        except Exception as e:
            raise RuntimeError("Unable to find bridge for network {}: {}".format(network, str(e)))
        if len(interfaces) != expected:
            raise RuntimeError("Expected {} interface(s) for network {} got {}".format(expected, network, len(interfaces)))
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

    def memory(self, tid):
        try:
            filename = 'logs/{}-memory.txt'.format(tid)
            output = open(filename, 'w')
            output.write("# Memory usage [%]\n")
            output.flush()
            self.workers.append(subprocess.Popen(['./scripts/memory_usage.sh'], stdout=output))
        except:
            raise RuntimeError('Failed to start ./scripts/memory_usage.sh')

    def cpu(self, tid):
        try:
            filename = 'logs/{}-cpu.txt'.format(tid)
            output = open(filename, 'w')
            output.write("# CPU usage [%]\n")
            output.flush()
            self.workers.append(subprocess.Popen(['./scripts/cpu_usage.sh'], stdout=output))
        except:
            raise RuntimeError('Failed to start ./scripts/cpu_usage.sh')

    def corruption(self, comm, corruptions, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for corruption in corruptions:
            tid = "{}-corruption-{:02d}".format(comm, corruption)
            tc = "netem corrupt {}%".format(corruption)
            title = "Corruption {}% ({})".format(corruption, comm.upper())
            self.run(comm, tid, tc, title, corruption, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Corruption rate [%]"
                prefix = "{}-corruption-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                if len(logs.keys()) > 1:
                    plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                    plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def reorder(self, comm, reorders, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for reorder in reorders:
            tid = "{}-reorder-{:02d}".format(comm, reorder)
            tc = "netem reorder {}% delay 25ms".format(reorder)
            title = "Reorders {}% ({})".format(reorder, comm.upper())
            self.run(comm, tid, tc, title, reorder, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Reordering rate [%]"
                prefix = "{}-reorder-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                if len(logs.keys()) > 1:
                    plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                    plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def duplication(self, comm, dups, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for dup in dups:
            tid = "{}-duplication-{:02d}".format(comm, dup)
            tc = "netem duplicate {}%".format(dup)
            title = "Duplication {}% ({})".format(dup, comm.upper())
            self.run(comm, tid, tc, title, dup, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Duplication rate [%]"
                prefix = "{}-duplication-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                if len(logs.keys()) > 1:
                    plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                    plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def limit(self, comm, limits, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for limit in limits:
            tid = "{}-limit-{:04d}kbit".format(comm, limit)
            tc = "tbf rate {}kbit burst 32kbit latency 500ms".format(limit)
            title = "Limit {}kbit ({})".format(limit, comm.upper())
            self.run(comm, tid, tc, title, limit, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Throughput limit [kbit]"
                prefix = "{}-limit-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                if len(logs.keys()) > 1:
                    plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                    plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def loss(self, comm, losses, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for loss in losses:
            tid = "{}-loss-{:02d}".format(comm, loss)
            tc = "netem loss {}%".format(loss)
            title = "Loss {}% ({})".format(loss, comm.upper())
            self.run(comm, tid, tc, title, loss, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Packet loss rate [%]"
                prefix = "{}-loss-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                if len(logs.keys()) > 1:
                    plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                    plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def delay(self, comm, delays, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for delay in delays:
            tid = "{}-delay-{:04d}".format(comm, delay)
            tc = "netem delay {}ms".format(delay)
            title = "Delay {}ms ({})".format(delay, comm.upper())
            self.run(comm, tid, tc, title, delay, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Packet delay [ms]"
                prefix = "{}-delay-{}".format(comm, cmd)
                self.extract(logs, prefix, cmd)
                if len(logs.keys()) > 1:
                    plotter.lostPackets("{}-lost-packets".format(prefix), cmd, xlabel)
                    plotter.firstReceived("{}-first-received".format(prefix), cmd, xlabel)
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def scalability(self, comm, numbers, skip, mean):
        logs = Logs()
        plotter = Plotter()
        for count in numbers:
            tid = "{}-scalability-{:02d}".format(comm, count)
            title = "Scalability - {} nodes ({})".format(count, comm.upper())
            self.run(comm + "_scalability", tid, count, title, count, skip, logs, mean)
        if len(logs.keys()) > 0:
            for cmd in self.commands:
                xlabel = "Number of nodes"
                prefix = "{}-scalability-{}".format(comm, cmd)
                logs.extractThroughput('data/{}-throughput.dat'.format(prefix), cmd)
                logs.extractLatency('data/{}-latency.dat'.format(prefix), cmd)
                if len(logs.keys()) > 1:
                    plotter.throughput("{}-throughput".format(prefix), cmd, xlabel)
                    plotter.latency("{}-latency".format(prefix), cmd, xlabel)

    def extract(self, logs, prefix, cmd):
        logs.extractLostPackets('data/{}-lost-packets.dat'.format(prefix), cmd)
        logs.extractFirstReceived('data/{}-first-received.dat'.format(prefix), cmd)
        logs.extractThroughput('data/{}-throughput.dat'.format(prefix), cmd)
        logs.extractLatency('data/{}-latency.dat'.format(prefix), cmd)

    def run(self, comm, tid, tc, prefix, value, skip, logs, mean):
        plotter = Plotter()
        if mean:
            console = glob.glob("results/*/logs/{}-console.txt".format(tid))
            robot = glob.glob("results/*/logs/{}-robot.txt".format(tid))
            if len(console) != len(robot):
                print('Missing logs for {}'.format(tid), flush=True, file=sys.stderr)
                return
            for i in range(len(console)):
                logs.parse(robot[i], value, "robot", i)
                logs.parse(console[i], value, "console", i)
            print("Found {} samples for {}".format(len(console), tid), flush=True)
            return
        elif skip:
            print("Using results from: {}".format(tid), flush=True)
        else:
            try:
                self.execute(comm, tid, tc)
            except Exception as e:
                print('Test {} failed: {}'.format(tid, str(e)), flush=True, file=sys.stderr)
                return
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
        if comm== "ros1":
            self.ros1(tid, tc)
        elif comm == "ros2opensplice":
            self.ros2opensplice(tid, tc)
        elif comm == "ros2fastrtps":
            self.ros2fastrtps(tid, tc)
        elif comm == "ros2connext":
            self.ros2connext(tid, tc)
        elif comm == "opensplice":
            self.opensplice(tid, tc)
        elif comm == "ros1bridge":
            self.ros1bridge(tid, tc)
        elif comm == "ros1_scalability":
            self.ros1_scalability(tid, tc)
        elif comm == "ros2opensplice_scalability":
            self.ros2opensplice_scalability(tid, tc)
        elif comm == "ros2connext_scalability":
            self.ros2connext_scalability(tid, tc)
        elif comm == "ros2fastrtps_scalability":
            self.ros2fastrtps_scalability(tid, tc)
        else:
            raise RuntimeError("Not implemented: " + comm)
        os.rename('logs/robot.txt', 'logs/{}-robot.txt'.format(tid))
        os.rename('logs/console.txt', 'logs/{}-console.txt'.format(tid))

    def ros1(self, tid, tc):
        master_id = subprocess.Popen("./scripts/start_ros1_master.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        master_if = self.interfaces("ros1", 1)[0]
        robot_id = subprocess.Popen("./scripts/start_ros1_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros1_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        try:
            interfaces = self.interfaces("ros1", 3)
            for interface in interfaces:
                if interface != master_if:
                    self.tc(interface, tc)
                    self.tcpdump(interface, tid)
        except Exception as e:
            self.kill_stop_remove([robot_id, console_id, master_id])
            raise e
        self.wait_kill_remove([robot_id, console_id])
        self.docker.stop(master_id)
        self.docker.remove_container(master_id)

    def ros2opensplice(self, tid, tc):
        robot_id = subprocess.Popen("./scripts/start_ros2opensplice_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros2opensplice_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        try:
            interfaces = self.interfaces("ros2opensplice", 2)
            for interface in interfaces:
                self.tc(interface, tc)
                self.tcpdump(interface, tid)
        except Exception as e:
            self.kill_stop_remove([robot_id, console_id])
            raise e
        self.wait_kill_remove([robot_id, console_id])

    def ros2fastrtps(self, tid, tc):
        robot_id = subprocess.Popen("./scripts/start_ros2fastrtps_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros2fastrtps_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        try:
            interfaces = self.interfaces("ros2fastrtps", 2)
            for interface in interfaces:
                self.tc(interface, tc)
                self.tcpdump(interface, tid)
        except Exception as e:
            self.kill_stop_remove([robot_id, console_id])
            raise e
        self.wait_kill_remove([robot_id, console_id])

    def ros2connext(self, tid, tc):
        robot_id = subprocess.Popen("./scripts/start_ros2connext_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros2connext_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        try:
            interfaces = self.interfaces("ros2connext", 2)
            for interface in interfaces:
                self.tc(interface, tc)
                self.tcpdump(interface, tid)
        except Exception as e:
            self.kill_stop_remove([robot_id, console_id])
            raise e
        self.wait_kill_remove([robot_id, console_id])

    def opensplice(self, tid, tc):
        robot_id = subprocess.Popen("./scripts/start_opensplice_robot.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_opensplice_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        try:
            interfaces = self.interfaces("opensplice", 2)
            for interface in interfaces:
                self.tc(interface, tc)
                self.tcpdump(interface, tid)
        except Exception as e:
            self.kill_stop_remove([robot_id, console_id])
            raise e
        self.wait_kill_remove([robot_id, console_id])

    def ros1bridge(self, tid, tc):
        master_id = subprocess.Popen("./scripts/start_ros1_master.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        time.sleep(5)
        bridge_id = subprocess.Popen("./scripts/start_ros1_bridge.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        skip = self.interfaces("ros1", 2)
        robot_id = subprocess.Popen("./scripts/start_ros2opensplice_robot_bridge.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        console_id = subprocess.Popen("./scripts/start_ros1_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        try:
            interfaces = self.interfaces("ros1", 4)
            for interface in interfaces:
                if not interface in skip:
                    self.tc(interface, tc)
                    self.tcpdump(interface, tid)
        except Exception as e:
            self.kill_stop_remove([robot_id, console_id, master_id, bridge_id])
            raise e
        self.wait_kill_remove([robot_id, console_id])
        self.docker.stop(master_id)
        self.docker.remove_container(master_id)
        self.docker.stop(bridge_id)
        self.docker.remove_container(bridge_id)

    def ros1_scalability(self, tid, count):
        ids = []
        self.cpu(tid)
        self.memory(tid)
        time.sleep(30)
        master_id = subprocess.Popen("./scripts/start_ros1_master.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip()
        ids.append(subprocess.Popen("./scripts/start_ros1_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        for ip in range(10, 10 + count):
            ids.append(subprocess.Popen("./scripts/start_ros1_scalability_robot.sh {}".format(ip), shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        self.wait_kill_remove(ids, 30)
        self.docker.stop(master_id)
        self.docker.remove_container(master_id)
        os.rename('logs/robot-{}.txt'.format(10 + count - 1), 'logs/robot.txt')

    def ros2connext_scalability(self, tid, count):
        ids = []
        self.cpu(tid)
        self.memory(tid)
        time.sleep(30)
        ids.append(subprocess.Popen("./scripts/start_ros2connext_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        for ip in range(10, 10 + count):
            ids.append(subprocess.Popen("./scripts/start_ros2connext_scalability_robot.sh {}".format(ip), shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        self.wait_kill_remove(ids, 30)
        os.rename('logs/robot-{}.txt'.format(10 + count - 1), 'logs/robot.txt')

    def ros2opensplice_scalability(self, tid, count):
        ids = []
        self.cpu(tid)
        self.memory(tid)
        time.sleep(30)
        ids.append(subprocess.Popen("./scripts/start_ros2opensplice_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        for ip in range(10, 10 + count):
            ids.append(subprocess.Popen("./scripts/start_ros2opensplice_scalability_robot.sh {}".format(ip), shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        self.wait_kill_remove(ids, 30)
        os.rename('logs/robot-{}.txt'.format(10 + count - 1), 'logs/robot.txt')

    def ros2fastrtps_scalability(self, tid, count):
        ids = []
        self.cpu(tid)
        self.memory(tid)
        time.sleep(30)
        ids.append(subprocess.Popen("./scripts/start_ros2fastrtps_console.sh", shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        for ip in range(10, 10 + count):
            ids.append(subprocess.Popen("./scripts/start_ros2fastrtps_scalability_robot.sh {}".format(ip), shell = True, stdout=subprocess.PIPE).stdout.read().decode("utf-8").rstrip())
        self.wait_kill_remove(ids, 30)
        os.rename('logs/robot-{}.txt'.format(10 + count - 1), 'logs/robot.txt')

