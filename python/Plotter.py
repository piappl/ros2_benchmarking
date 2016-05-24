import subprocess

class Plotter:
    def compareTimes(self, tid, title):
        params = [ "title='{}'".format(title) ]
        params.append("input1='data/{}-published.dat'".format(tid))
        params.append("input2='data/{}-received.dat'".format(tid))
        params.append("output='graphs/{}.png'".format(tid))
        self.call('compare-times.plt', params)

    def lostPackets(self, tid, title):
        params = [ "title='{}'".format(title) ]
        params.append("input='data/{}-lost-packets.dat'".format(tid))
        params.append("output='graphs/{}-lost-packets.png'".format(tid))
        self.call('lost-packets.plt', params)

    def lostPacketsEstablished(self, tid, title):
        params = [ "title='{}'".format(title) ]
        params.append("input='data/{}-lost-packets-established.dat'".format(tid))
        params.append("output='graphs/{}-lost-packets-established.png'".format(tid))
        self.call('lost-packets-established.plt', params)

    def publishedHistogram(self, tid, title):
        params = [ "title='{}'".format(title) ]
        params.append("input='data/{}-published.dat'".format(tid))
        params.append("output='graphs/{}-published.png'".format(tid))
        params.append("label='Packets published'")
        params.append("legend='Packets published by console'")
        self.call('histogram.plt', params)

    def receivedHistogram(self, tid, title):
        params = [ "title='{}'".format(title) ]
        params.append("input='data/{}-received.dat'".format(tid))
        params.append("output='graphs/{}-received.png'".format(tid))
        params.append("label='Packets received'")
        params.append("legend='Packets received by robot'")
        self.call('histogram.plt', params)

    def firstReceived(self, tid, title):
        params = [ "title='{}'".format(title) ]
        params.append("input='data/{}-first-received.dat'".format(tid))
        params.append("output='graphs/{}-first-received.png'".format(tid))
        self.call('first-received.plt', params)

    def call(self, script, params):
        command = 'gnuplot -e "{}" gnuplot/{}'.format("; ".join(params), script)
        fh = open('graphs/replot.sh', "a")
        fh.write(command + "\n")
        subprocess.call(command, shell = True)

