import sys, os, re, datetime

class Logs:
    logs = None
    missing = "NA"

    def __init__(self):
        self.logs = {}

    def parse(self, filename, key, node, count = 0):
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
                self.logs[key] = []
            while len(self.logs[key]) <= count:
                self.logs[key].append({})
            self.logs[key][count][node] = logs
        else:
            print("Failed to parse logs: {}".format(filename), flush=True, file=sys.stderr)

    def keys(self):
        return self.logs.keys()

    def extractCommand(self, filename, key, command, direction):
        if key in self.logs and len(self.logs[key]) == 1 and "robot" in self.logs[key][0] and "console" in self.logs[key][0]:
            fh = open(filename, "a")
            for line in self.logs[key][0]["robot"]["Packets"] + self.logs[key][0]["console"]["Packets"]:
                if (line[direction] and line['Command'] == command):
                    fh.write("{}\n".format((line['Datetime'] - self.logs[key][0]["console"]["Teststart"]).total_seconds()))
        else:
            print("Missing data for: {}-{}-{}".format(key, command, direction), flush=True, file=sys.stderr)

    def extractLostPackets(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            count = 0
            sent = 0
            received = 0
            for sample in self.logs[key]:
                if "robot" in sample and "console" in sample:
                    count = count + 1
                    for line in sample['console']["Packets"] + sample['robot']["Packets"]:
                        if line['Command'] == cmd:
                            if line["Published"]:
                                sent += 1
                            elif line["Received"]:
                                received += 1
            if count > 0:
                fh.write('{} {} {}\n'.format(key, received / count, sent / count))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {} {}\n'.format(key, self.missing, self.missing))


    def extractThroughput(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            count = 0
            total = 0
            for sample in self.logs[key]:
                if "robot" in sample and "console" in sample:
                    period = None
                    bytes = 0
                    for name in [ "console", "robot" ]:
                        node = sample[name]
                        period = (node["Testfinished"] - node["Teststart"]).total_seconds()
                        for line in node["Packets"]:
                            if line['Command'] == cmd and line["Received"]:
                                if line["Datetime"] > node["Teststart"] and line["Datetime"] < node["Testfinished"]:
                                    bytes += int(line["Size"])
                    total = total + (bytes / period)
                    count = count + 1
            if count > 0:
                fh.write('{} {}\n'.format(key, total / count))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {}\n'.format(key, self.missing))

    def extractLatency(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            count = 0
            latency = 0
            for sample in self.logs[key]:
                if "robot" in sample and "console" in sample:
                    packets = {}
                    for name in [ "console", "robot" ]:
                        for line in sample[name]["Packets"]:
                            if line['Command'] == cmd:
                                if not line['Id'] in packets:
                                    packets[line['Id']] = {}
                                packets[line['Id']][line['Type']] = line['Datetime']
                    for pid in packets:
                        if 'RECEIVED' in packets[pid] and 'PUBLISHING' in packets[pid]:
                            count += 1
                            latency += (packets[pid]['RECEIVED'] - packets[pid]['PUBLISHING']).total_seconds() * 1000
            if count > 0:
                fh.write('{} {}\n'.format(key, latency / count))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {}\n'.format(key, self.missing))

    def extractFirstReceived(self, filename, cmd):
        fh = open(filename, "w")
        for key in sorted(self.logs):
            count = 0
            total = 0
            for sample in self.logs[key]:
                if "robot" in sample and "console" in sample:
                    published = None
                    received = None
                    for line in sample['console']["Packets"] + sample['robot']["Packets"]:
                        if line['Command'] == cmd:
                            if line["Published"] and published == None:
                                published = line["Datetime"]
                            elif line["Received"] and received == None:
                                received = line["Datetime"]
                    if received != None and published != None:
                        total = total + (received - published).total_seconds()
                        count = count + 1
            if count > 0:
                fh.write('{} {}\n'.format(key, total / count))
            else:
                print("Missing data for: {}".format(filename), flush=True, file=sys.stderr)
                fh.write('{} {}\n'.format(key, self.missing))
