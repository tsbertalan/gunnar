'''An attempt at writing a script that launches all lauchfiles,
both on sigurd and gunnar.'''

import subprocess
from time import sleep
from os import system


def popen(*args, **kwargs):
    print 'Popen: %s, %s' % (args, kwargs)
    return subprocess.Popen(*args, **kwargs)


def prepend(l, *items):
    l.reverse()
    items = list(items)
    items.reverse()
    for i in items:
        l.append(i)
    l.reverse()


procs = []
def runOnGunnar(args):
    args = list(args)
    prepend(args, 'ssh', 'pi@gunnar.tomsb.net')
    proc = popen(args)
    procs.append(proc)
    return proc

screens = []
class Screen(object):
     
    def __init__(self, screenName, commandArguments, remote=True):
        self.screenName = screenName
        commandArguments = list(commandArguments)
        prepend(commandArguments, 'screen', '-dmLS', screenName)
        if remote:
            self.process = runOnGunnar(commandArguments)
        else:
            self.process = popen(commandArguments)
        screens.append(self)
        self.remote = remote
    
    def kill(self):
        print 'Killing screen "%s".' % self.screenName
        if self.remote:
            killer = runOnGunnar(['screen', '-X', '-S', self.screenName, 'quit'])
        else:
            killer = popen(['screen', '-X', '-S', self.screenName, 'quit'])


# sync gunnar
system('ssh pi@gunnar.tomsb.net unison workspace')

# Start screens.
Screen('hardware', ['roslaunch', 'gunnar', 'hardware_drivers.launch'])
sleep(1.0)
# Next command fails because LD_LIBRARY_PATH isn't set.
# http://stackoverflow.com/questions/1099981/why-cant-python-find-shared-objects-that-are-in-directories-in-sys-path
Screen('rab', ['roslaunch', 'gunnar', 'ros_arduino_bridge.launch'])

# sleep(4.0)
# Screen('slam', ['roslaunch', 'gunnar', 'slam.launch'])
# sleep(1.0)
# Screen('slam', ['roslaunch', 'gunnar', 'path_planning.launch'])
sleep(1.0)
popen(['rviz'])



while True:
    try:
        sleep(1.0)
    except KeyboardInterrupt:
        for screen in screens:
            screen.kill()
            sleep(2.0)
        
        for proc in procs:
            proc.terminate()
        break
    
    
