#!/usr/bin/env python
from time import sleep, time
import matplotlib.pyplot as plt
import numpy as np
from os import getcwd


def record(fname='data.npz', nsteps=200, rate=10.0):
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    import rospy
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('encoderCountsOverTime')
    
    rospy.logerr('Running in %s.', getcwd())
    
    twist = Twist()
    
    def getTicks():
        ticksStart = rospy.wait_for_message('/encl', Int32)
        return ticksStart.data
    
    startTime = time() 
    
    times = []
    ticks = []
    rospy.logerr('Iterating at %s Hz for %s steps.', rate, nsteps)
    for i in range(nsteps):
        times.append(time())
        ticks.append(getTicks())
        sleep(1./rate)
        
    rospy.logerr('Saving %s.', fname)
    np.savez(fname, times=times, ticks=ticks)
    return fname


def goalTimes(nsecs=2):
    freqs = 1, 2, 10, 100, 1000, 5000, 10000, 20000
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
    
    fig, axes = plt.subplots(ncols=3, figsize=(24, 6))
    
    ax = axes[0]
    ax.plot(times, ticks, 'ko')
    ax.set_xlabel('time')
    ax.set_ylabel('encoder distance')
    
    
    ax = axes[1]
    y = np.diff(ticks)
    ax.plot(times[:-1], np.abs(y), 'ko')
    ax.set_yscale('log')
    ax.set_xlabel('time')
    ax.set_ylabel('$|$ $\Delta$ encoder distance$|$')
    
    ax = axes[2]
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
#     plot(record())
    plot()
    plt.show()
    