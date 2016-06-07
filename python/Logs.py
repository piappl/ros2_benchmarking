import re, datetime

class Logs:
    teststart = None

    def __init__(self):
        self.teststart = datetime.datetime.now()

    def parse(self, filename):
        log = []
        f = open(filename, 'r')
        for line in f.readlines():
           result = re.match("^(.+) BENCHMARK! (\w+) (\w+): id=(\d+)", line);
           if result:
               data = {'Time': result.group(1), 'Type': result.group(2), 'Command': result.group(3), 'Id': result.group(4)}
               data["Received"] = 1 if result.group(2) == "RECEIVED" else 0
               data["Published"] = 1 if result.group(2) == "PUBLISHING" else 0
               data["Datetime"] = datetime.datetime.strptime(data["Time"], '%Y-%m-%d %H:%M:%S.%f')
               log.append(data);
        return log

    def extractCommand(self, filename, command, direction, log):
        fh = open(filename, "a")
        for line in log:
            if (line[direction] and line['Command'] == command):
                fh.write("{}\n".format((line['Datetime'] - self.teststart).total_seconds()))

    def extractLostPackets(self, filename, cmd, logs):
        fh = open(filename, "w")
        for log in logs:
            sent = 0
            received = 0
            for line in log['Console'] + log['Robot']:
                if line['Command'] == cmd:
                    if line["Published"]:
                        sent += 1
                    elif line["Received"]:
                        received += 1
            fh.write('{} {} {}\n'.format(log['Value'], received, sent))

    def extractLostPacketsEstablished(self, filename, logs):
        fh = open(filename, "w")
        for log in logs:
            sent = 0
            received = 0
            first = None
            for line in log['Robot']:
                if (line["Received"] and line['Command'] == 'cmd_vel'):
                    received += 1
                    if first == None:
                        first = line["Datetime"]
            for line in log['Console']:
                if (line["Published"] and line['Command'] == 'cmd_vel'):
                    if received == 0 or line["Datetime"] > first:
                        sent += 1
            fh.write('{} {}\n'.format(log['Value'], sent - received))

    def extractFirstReceived(self, filename, cmd, logs):
        fh = open(filename, "w")
        for log in logs:
            published = None
            received = None
            for line in log['Console'] + log['Robot']:
                if line['Command'] == cmd:
                    if line["Published"]:
                        published = line["Datetime"]
                    elif line["Received"]:
                        received = line["Datetime"]
            if received != None and published != None:
                fh.write('{} {}\n'.format(log['Value'], (received - published).total_seconds()))
