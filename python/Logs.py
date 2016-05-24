import re, datetime

class Logs:
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

    def extractCmdVel(self, filename, direction, log, teststart):
        fh = open(filename, "w")
        for line in log:
            if (line[direction] and line['Command'] == 'cmd_vel'):
                fh.write("{}\n".format((line['Datetime'] - teststart).total_seconds()))

    def extractLostPackets(self, filename, logs):
        fh = open(filename, "w")
        for log in logs:
            sent = 0
            received = 0
            for line in log['Console']:
                if (line["Published"] and line['Command'] == 'cmd_vel'):
                    sent += 1
            for line in log['Robot']:
                if (line["Received"] and line['Command'] == 'cmd_vel'):
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
                    if line["Datetime"] > first:
                        sent += 1
            fh.write('{} {}\n'.format(log['Value'], sent - received))

    def extractFirstReceived(self, filename, logs):
        fh = open(filename, "w")
        for log in logs:
            published = None
            received = None
            for line in log['Console']:
                if (line["Published"] and line['Command'] == 'cmd_vel'):
                    published = line["Datetime"]
                    break
            for line in log['Robot']:
                if (line["Received"] and line['Command'] == 'cmd_vel'):
                    received = line["Datetime"]
                    break
            fh.write('{} {}\n'.format(log['Value'], (received - published).total_seconds()))
