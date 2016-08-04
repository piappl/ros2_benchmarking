import subprocess, os, sys

class Plotter:

    def compareTimes(self, tid, cmd, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input1", "data/{}-{}-pub.dat".format(tid, cmd))
            self.input(params, "input2", "data/{}-{}-rec.dat".format(tid, cmd))
            params.append("output='graphs/times/{}-{}.eps'".format(tid, cmd))
            self.call('compare-times.plt', params)
        except:
            pass

    def lostPackets(self, filename, cmd, xlabel):
        try:
            params = [ "title='{} message: lost packets'".format(cmd), "xlabel='{}'".format(xlabel) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/lost-packets/{}.eps'".format(filename))
            self.call('lost-packets.plt', params)
        except:
            pass

    def publishedHistogram(self, tid, cmd, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}-{}-pub.dat".format(tid, cmd))
            params.append("output='graphs/histograms/{}-{}-pub.eps'".format(tid, cmd))
            params.append("label='Packets published'")
            params.append("legend='Packets published by console'")
            self.call('histogram.plt', params)
        except:
            pass

    def receivedHistogram(self, tid, cmd, title):
        try:
            params = [ "title='{}'".format(title) ]
            self.input(params, "input", "data/{}-{}-rec.dat".format(tid, cmd))
            params.append("output='graphs/histograms/{}-{}-rec.eps'".format(tid, cmd))
            params.append("label='Packets received'")
            params.append("legend='Packets received by robot'")
            self.call('histogram.plt', params)
        except:
            pass

    def firstReceived(self, filename, cmd, xlabel):
        try:
            params = [ "title='{} message: first packet received'".format(cmd), "xlabel='{}'".format(xlabel) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/first-received/{}.eps'".format(filename))
            self.call('first-received.plt', params)
        except:
            pass

    def throughput(self, filename, cmd, xlabel):
        try:
            params = [ "title='{} message: throughput'".format(cmd), "xlabel='{}'".format(xlabel) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/throughput/{}.eps'".format(filename))
            self.call('throughput.plt', params)
        except:
            pass

    def latency(self, filename, cmd, xlabel):
        try:
            params = [ "title='{} message: latency'".format(cmd), "xlabel='{}'".format(xlabel) ]
            self.input(params, "input", "data/{}.dat".format(filename))
            params.append("output='graphs/latency/{}.eps'".format(filename))
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
            print("File does not exist: {}".format(filename), flush=True, file=sys.stderr)
            raise RuntimeError
        if os.path.getsize(filename) == 0:
            print("File is empty: {}".format(filename), flush=True, file=sys.stderr)
            raise RuntimeError
        params.append("{}='{}'".format(name, filename))

