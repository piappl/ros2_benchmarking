import subprocess, os, sys

class Plotter:

    def compareTimes(self, tid, cmd, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input1", "data/{}-{}-pub.dat".format(tid, cmd))
            self.input(params, "input2", "data/{}-{}-rec.dat".format(tid, cmd))
            params.append("output='graphs/times/{}-{}.png'".format(tid, cmd))
            self.call('compare-times.plt', params)
        except:
            pass

    def lostPackets(self, filename, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/lost-packets/{}.png'".format(filename))
            self.call('lost-packets.plt', params)
        except:
            pass

    def lostPacketsEstablished(self, tid, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}-lost-packets-established.dat".format(tid))
            params.append("output='graphs/{}-lost-packets-established.png'".format(tid))
            self.call('lost-packets-established.plt', params)
        except:
            pass

    def publishedHistogram(self, tid, cmd, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}-{}-pub.dat".format(tid, cmd))
            params.append("output='graphs/histograms/{}-{}-pub.png'".format(tid, cmd))
            params.append("label='Packets published'")
            params.append("legend='Packets published by console'")
            self.call('histogram.plt', params)
        except:
            pass

    def receivedHistogram(self, tid, cmd, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}-{}-rec.dat".format(tid, cmd))
            params.append("output='graphs/histograms/{}-{}-rec.png'".format(tid, cmd))
            params.append("label='Packets received'")
            params.append("legend='Packets received by robot'")
            self.call('histogram.plt', params)
        except:
            pass

    def firstReceived(self, filename, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/first-received/{}.png'".format(filename))
            self.call('first-received.plt', params)
        except:
            pass

    def throughput(self, filename, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/throughput/{}.png'".format(filename))
            self.call('throughput.plt', params)
        except:
            pass

    def latency(self, filename, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/latency/{}.png'".format(filename))
            self.call('latency.plt', params)
        except:
            pass

    def call(self, script, params):
        command = 'gnuplot -e "{}" gnuplot/{}'.format("; ".join(params), script)
        fh = open('graphs/replot.sh', "a")
        fh.write(command + "\n")
        subprocess.call(command, shell = True)

    def input(self, params, name, filename):
        if not os.path.isfile(filename):
            print("File does not exist: {}".format(filename), file=sys.stderr)
            raise RuntimeError
        if os.path.getsize(filename) == 0:
            print("File is empty: {}".format(filename), file=sys.stderr)
            raise RuntimeError
        params.append("{}='{}'".format(name, filename))

