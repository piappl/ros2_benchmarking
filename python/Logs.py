import sys, os, re, datetime

class Logs:
    logs = {}

    def parse(self, filename, key, node):
        if not os.path.isfile(filename):
            print("File does not exist: {}".format(filename), file=sys.stderr)
            return
        f = open(filename, 'r')
        logs = { 'Packets': [] }
        for line in f.readlines():
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

        if "Nodestart" in logs and "Teststart" in logs and "Testfinished" in logs:
            if not key in self.logs:
                self.logs[key] = { node: logs }
            else:
                self.logs[key][node] = logs
        else:
            print("Failed to parse logs: {}".format(filename), file=sys.stderr)

    def keys(self):
        return self.logs.keys()

    def extractCommand(self, filename, key, command, direction):
        if key in self.logs and "robot" in self.logs[key] and "console" in self.logs[key]:
            fh = open(filename, "a")
            for line in self.logs[key]["robot"]["Packets"] + self.logs[key]["console"]["Packets"]:
                if (line[direction] and line['Command'] == command):
                    fh.write("{}\n".format((line['Datetime'] - self.logs[key]["console"]["Teststart"]).total_seconds()))
        else:
            print("Missing data for: {}-{}-{}".format(key, command, direction), file=sys.stderr)

    def extractLostPackets(self, filename, cmd):
        fh = open(filename, "w")
        for key in self.logs:
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
                print("Missing data for: {}".format(filename), file=sys.stderr)

    def extractThroughput(self, filename, cmd):
        fh = open(filename, "w")
        for key in self.logs:
            if "robot" in self.logs[key] and "console" in self.logs[key]:
                period = None
                bytes = 0
                for name in [ "console", "robot" ]:
                    node = self.logs[key][name]
                    period = (node["Testfinished"] - node["Teststart"]).total_seconds()
                    for line in node["Packets"]:
                        if line['Command'] == cmd and line["Published"]:
                            if line["Datetime"] > node["Teststart"] and line["Datetime"] < node["Testfinished"]:
                                bytes += int(line["Size"])
                    if bytes > 0:
                        fh.write('{} {}\n'.format(key, bytes / period))
                        bytes = 0
            else:
                print("Missing data for: {}".format(filename), file=sys.stderr)

    def extractFirstReceived(self, filename, cmd):
        fh = open(filename, "w")
        for key in self.logs:
            if "robot" in self.logs[key] and "console" in self.logs[key]:
                published = None
                received = None
                for line in self.logs[key]['console']["Packets"] + self.logs[key]['robot']["Packets"]:
                    if line['Command'] == cmd:
                        if line["Published"]:
                            published = line["Datetime"]
                        elif line["Received"]:
                            received = line["Datetime"]
                if received != None and published != None:
                    fh.write('{} {}\n'.format(key, (received - published).total_seconds()))
            else:
                print("Missing data for: {}".format(filename), file=sys.stderr)


#    def extractLostPacketsEstablished(self, filename, logs):
#        fh = open(filename, "w")
#        for log in logs:
#            sent = 0
#            received = 0
#            first = None
#            for line in log['Robot']:
#                if (line["Received"] and line['Command'] == 'cmd_vel'):
#                    received += 1
#                    if first == None:
#                        first = line["Datetime"]
#            for line in log['Console']:
#                if (line["Published"] and line['Command'] == 'cmd_vel'):
#                    if received == 0 or line["Datetime"] > first:
#                        sent += 1
#            fh.write('{} {}\n'.format(log['Value'], sent - received))

