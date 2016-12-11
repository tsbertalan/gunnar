#!/usr/bin/env python
from time import sleep, time
import matplotlib.pyplot as plt
import numpy as np
from os import getcwd


def record(fname='data.npz', nsteps=256, rate=2.0):
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    import rospy
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('encoderCountsOverTime')
    
    rospy.logwarn('Running in %s.', getcwd())
    
    twist = Twist()
    
    def getTicks():
        ticksStart = rospy.wait_for_message('/encl', Int32)
        return ticksStart.data
    
    startTime = time() 
    
    times = []
    ticks = []
    rospy.logwarn('Iterating at %s Hz for %s steps.', rate, nsteps)
    for i in range(nsteps):
        if not i % rate:
            rospy.logwarn('Step i = %d.', i)
        times.append(time())
        ticks.append(getTicks())
        sleep(1./rate)
        
    rospy.logwarn('Saving %s.', fname)
    np.savez(fname, times=times, ticks=ticks)
    return fname


def goalTimes(nsecs=8):
    freqs = 31, 100, 1000, 5000, 10000, 20000, 40000, 60000
    t = 0.0
    gtimes = [t]
    gfreqs = []
    for freq in freqs:
        periodUs = 1.0 / freq * 1000 * 1000
        ncycs = nsecs * freq
        for cyc in range(ncycs):
            t += periodUs * 1e-6
            gtimes.append(t)
            gfreqs.append(freq)
    return np.array(gtimes), np.array(gfreqs)
    
    
def plot(fname='data.npz'):
    plotName = fname + '.png'
    data = np.load(fname)
    print 'Loaded %s. Got keys: %s' % (fname, data.keys())
    
    times = np.array(data['times'])
    times -= min(times)
    ticks = data['ticks']
    
    elapsedTime = np.diff(times)
    elapsedTicks = np.abs(np.diff(ticks))
    avTimePerTick = elapsedTime / elapsedTicks
    freqs = 1.0 / avTimePerTick
    
    fig, axes = plt.subplots(ncols=2, figsize=(24, 8))
    
    ax = axes[0]
    ax.plot(times, ticks, 'ko')
    ax.set_xlabel('time')
    ax.set_ylabel('encoder distance')
    
    ax = axes[1]
    y = freqs 
    ax.plot(times[:-1], y, 'ko', label='measured')
    
    gtimes, gfreqs = goalTimes()
    ax.scatter(gtimes[:-1], gfreqs, color='red', label='goal')
    
    ax.set_xlabel('time')
    ax.set_ylabel('average frequency\n'+'per %.2f-second sample' % np.mean(np.diff(times))
                  +'\n'+'(max=%.0f)' % max(y))
    
    ax.set_yscale('log')
    ax.legend(loc='lower right')
    
    fig.tight_layout()
    
    print 'Saving %s' % plotName
    fig.savefig(plotName)

    
if __name__ == '__main__':
    import socket
    if socket.gethostname().lower() == 'raspberrypi':
        record()
    else:
        plot()
        plt.show()
    