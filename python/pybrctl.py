import subprocess 

class BridgeException(Exception):
    pass

class Bridge(object):

    def __init__(self, name):
        """ Initialize a bridge object. """
        self.name = name

    def __str__(self):
        """ Return a string of the bridge name. """
        return self.name

    def __repr__(self):
        """ Return a representaion of a bridge object. """
        return "<Bridge: %s>" % self.name

    def addif(self, iname):
        """ Add an interface to the bridge """
        _runshell(['/sbin/brctl', 'addif', self.name, iname],
            "Could not add interface %s to %s." % (iname, self.name))

    def delif(self, iname):
        """ Delete an interface from the bridge. """
        _runshell(['/sbin/brctl', 'delif', self.name, iname],
            "Could not delete interface %s from %s." % (iname, self.name))

    def hairpin(self, port, val=True):
        """ Turn harpin on/off on a port. """ 
        if val: state = 'on' 
        else: state = 'off'
        _runshell(['/sbin/brctl', 'hairpin', self.name, port, state],
            "Could not set hairpin in port %s in %s." % (port, self.name))

    def stp(self, val=True):
        """ Turn STP protocol on/off. """
        if val: state = 'on' 
        else: state = 'off'
        _runshell(['/sbin/brctl', 'stp', self.name, state],
            "Could not set stp on %s." % self.name)

    def setageing(self, time):
        """ Set bridge ageing time. """
        _runshell(['/sbin/brctl', 'setageing', self.name, str(time)],
            "Could not set ageing time in %s." % self.name)

    def setbridgeprio(self, prio):
        """ Set bridge priority value. """
        _runshell(['/sbin/brctl', 'setbridgeprio', self.name, str(prio)],
            "Could not set bridge priority in %s." % self.name)
    
    def setfd(self, time):
        """ Set bridge forward delay time value. """
        _runshell(['/sbin/brctl', 'setfd', self.name, str(time)],
            "Could not set forward delay in %s." % self.name)

    def sethello(self, time):
        """ Set bridge hello time value. """
        _runshell(['/sbin/brctl', 'sethello', self.name, str(time)],
            "Could not set hello time in %s." % self.name)
   
    def setmaxage(self, time):
        """ Set bridge max message age time. """
        _runshell(['/sbin/brctl', 'setmaxage', self.name, str(time)],
            "Could not set max message age in %s." % self.name)
 
    def setpathcost(self, port, cost):
        """ Set port path cost value for STP protocol. """
        _runshell(['/sbin/brctl', 'setpathcost', self.name, port, str(cost)],
            "Could not set path cost in port %s in %s." % (port, self.name))
 
    def setportprio(self, port, prio):
        """ Set port priority value. """
        _runshell(['/sbin/brctl', 'setportprio', self.name, port, str(prio)],
            "Could not set priority in port %s in %s." % (port, self.name))

    def _show(self):
        """ Return a list of unsorted bridge details. """ 
        p = _runshell(['/sbin/brctl', 'show', self.name],
            "Could not show %s." % self.name)
        text = str(p.communicate()[0].decode("utf-8"))
        return text.split()[7:]

    def getid(self):
        """ Return the bridge id value. """
        return self._show()[1]

    def getifs(self):
        """ Return a list of bridge interfaces. """
        return self._show()[3:]

    def getstp(self):
        """ Return if STP protocol is enabled. """
        return self._show()[2] == 'yes'

    def showmacs(self):
        """ Return a list of mac addresses. """
        raise NotImplementedError()

    def showstp(self):
        """ Return STP information. """
        raise NotImplementedError()
           

class BridgeController(object):

    def addbr(self, name):
        """ Create a bridge a set the device up. """
        _runshell(['/sbin/brctl', 'addbr', name],
            "Could not create bridge %s." % name)
        _runshell(['/sbin/ip', 'link', 'set', 'dev', name, 'up'],
            "Could not set link up for %s." % name)
        return Bridge(name)

    def delbr(self, name):
        """ Set the device down and delete the bridge. """
        self.getbr(name) # Check if exists
        _runshell(['/sbin/ip', 'link', 'set', 'dev', name, 'down'],
            "Could not set link down for %s." % name)
        _runshell(['/sbin/brctl', 'delbr', name],
            "Could not delete bridge %s." % name)

    def showall(self):
        """ Return a list of all available bridges. """
        p = _runshell(['/sbin/brctl', 'show'],
            "Could not show bridges.")
        text = str(p.communicate()[0].decode("utf-8"))
        wlist = map(str.split, text.splitlines()[1:])
        brwlist = filter(lambda x: len(x) == 4, wlist)
        brlist = map(lambda x: x[0], brwlist)
        return map(Bridge, brlist)

    def getbr(self, name):
        """ Return a bridge object."""
        for br in self.showall():
            if br.name == name:
                return br
        raise BridgeException("Bridge does not exist.")


def _runshell(cmd, exception):
    """ Run a shell command. if fails, raise a proper exception. """
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if p.wait() != 0:
        raise BridgeException(exception)
    return p


