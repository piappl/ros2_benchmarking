import sys, os, re, datetime

class Logs:
    logs = None
    missing = "NA"

    def __init__(self):
        self.logs = {}

    def parse(self, filename, key, node):
        if not os.path.isfile(filename):
            print("File does not exist: {}".format(filename), flush=True, file=sys.stderr)
            return
        f = open(filename, 'r')
        logs = { 'Packets': [] }
        for line in f.readlines():
            try:
                result = re.match("^(.+) BENCHMARK! (\w+) (\w+): id=(\d+), size=(\d+)", line);
                if result:
                   data = {
                        'Time': result.group(1),
                        'Type': result.group(2),
                        'Command': result.group(3),
                        'Id': result.group(4),
                        'Size': result.group(5)
                   }
                   data["Received"] = 1 if result.group(2) == "RECEIVED" else 0
                   data["Published"] = 1 if result.group(2) == "PUBLISHING" else 0
                   data["Datetime"] = datetime.datetime.strptime(data["Time"], '%Y-%m-%d %H:%M:%S.%f')
                   logs["Packets"].append(data);

                result = re.match("^(.+) BENCHMARK! startNode: Initializing", line)
                if result:
                    logs["Nodestart"] = datetime.datetime.strptime(result.group(1), '%Y-%m-%d %H:%M:%S.%f')

                result = re.match("^(.+) BENCHMARK! TestRunner: Starting test", line)
                if result:
                    logs["Teststart"] = datetime.datetime.strptime(result.group(1), '%Y-%m-%d %H:%M:%S.%f')

                result = re.match("^(.+) BENCHMARK! TestRunner: Test finished", line)
                if result:
                    logs["Testfinished"] = datetime.datetime.strptime(result.group(1), '%Y-%m-%d %H:%M:%S.%f')
            except:
                print("Failed to parse: {}".format(line), flush=True, file=sys.stderr)
        if "Nodestart" in logs and "Teststart" in logs and "Testfinished" in logs:
            if not key in self.logs:
                self.logs[key] = { node: logs }
            else:
                self.logs[key][node] = logs
        else:
            print("Failed to parse logs: {}".format(filename), flush=True, file=sys.stderr)

    def keys(self):
        return self.logs.keys()

    def extractCommand(self, filename, key, command, direction):
        if key in self.logs and "robot" in self.logs[key] and "console" in self.logs[key]:
            fh = open(filename, "a")
            for line in self.logs[key]["robot"]["Packets"] + self.logs[key]["console"]["Packets"]:
                if (line[direction] and line['Command'] == command):
                    fh.write("{}\n".format((line['Datetime'] - self.logs[key]["console"]["Teststart"]).total_seconds()))
        else:
            print("Missing data for: {}-{}-{}".format(key, command, direction), flush=True, file=sys.stderr)

    def extractLostPackets(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            if "robot" in self.logs[key] and "console" in self.logs[key]:
                sent = 0
                received = 0
                for line in self.logs[key]['console']["Packets"] + self.logs[key]['robot']["Packets"]:
                    if line['Command'] == cmd:
                        if line["Published"]:
                            sent += 1
                        elif line["Received"]:
                            received += 1
                fh.write('{} {} {}\n'.format(key, received, sent))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {} {}\n'.format(key, self.missing, self.missing))

    def extractThroughput(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            if "robot" in self.logs[key] and "console" in self.logs[key]:
                period = None
                bytes = 0
                for name in [ "console", "robot" ]:
                    node = self.logs[key][name]
                    period = (node["Testfinished"] - node["Teststart"]).total_seconds()
                    for line in node["Packets"]:
                        if line['Command'] == cmd and line["Received"]:
                            if line["Datetime"] > node["Teststart"] and line["Datetime"] < node["Testfinished"]:
                                bytes += int(line["Size"])
                    if bytes > 0:
                        fh.write('{} {}\n'.format(key, bytes / period))
                        bytes = 0
                    else:
                        fh.write('{} {}\n'.format(key, self.missing))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {}\n'.format(key, self.missing))

    def extractLatency(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            if "robot" in self.logs[key] and "console" in self.logs[key]:
                packets = {}
                for name in [ "console", "robot" ]:
                    for line in self.logs[key][name]["Packets"]:
                        if line['Command'] == cmd:
                            if not line['Id'] in packets:
                                packets[line['Id']] = {}
                            packets[line['Id']][line['Type']] = line['Datetime']
                count = 0
                latency = 0
                for pid in packets:
                    if 'RECEIVED' in packets[pid] and 'PUBLISHING' in packets[pid]:
                        count += 1
                        latency += (packets[pid]['RECEIVED'] - packets[pid]['PUBLISHING']).total_seconds() * 1000
                if count > 0:
                    fh.write('{} {}\n'.format(key, latency / count))
                else:
                    fh.write('{} {}\n'.format(key, self.missing))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {}\n'.format(key, self.missing))

    def extractFirstReceived(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            if "robot" in self.logs[key] and "console" in self.logs[key]:
                published = None
                received = None
                for line in self.logs[key]['console']["Packets"] + self.logs[key]['robot']["Packets"]:
                    if line['Command'] == cmd:
                        if line["Published"] and published == None:
                            published = line["Datetime"]
                        elif line["Received"] and received == None:
                            received = line["Datetime"]
                if received != None and published != None:
                    fh.write('{} {}\n'.format(key, (received - published).total_seconds()))
                else:
                    fh.write('{} {}\n'.format(key, self.missing))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {}\n'.format(key, self.missing))

