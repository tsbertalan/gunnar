#!/usr/bin/env python
from time import sleep, time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.init_node('isrSpeedTest')

twist = Twist()
twist = Twist()
for v in 'xyz':
    setattr(twist.linear, v, 0.0)
    setattr(twist.angular, v, 0.0)

def go(spd):
    twist.linear.x = spd
    rospy.logwarn('Running motors forward at speed %s.', spd)
    publisher.publish(twist)

def getTicks():
    ticksStart = rospy.wait_for_message('/encr', Int32)
    return ticksStart.data

startTicks = getTicks()
startTime = time() 

setSpeed = 0.4
go(setSpeed)
goalElapsedTime = 8.0
sleep(goalElapsedTime)

endTicks = getTicks()
endTime = time()
go(0.0)

trueElapsed = endTime - startTime
revolutionsLUT = {   # Manually counted on previous runs.
    (0.1, 32.0): 12.25,
    (0.3, 8.0): 4.6,
    (0.4, 8.0): 5.0,
    }
revolutions = revolutionsLUT[(setSpeed, goalElapsedTime)]

wrn = rospy.logwarn
wrn('Revolutions: %s', revolutions)
wrn('RPM: %s', revolutions / trueElapsed * 60.0)
wrn('Ticks Elapsed: %s - %s = %s', endTicks, startTicks, endTicks-startTicks)
wrn('Seconds: %s - %s = %s ~= %s', endTime, startTime, trueElapsed, goalElapsedTime)
wrn('Seconds/Tick: %s', trueElapsed / (endTicks-startTicks))

def getExpectedTickTime(revs, elapsed):
    cprMot = 48.0 / 4.0
    gearRatio = 172.0
    cprWheel = cprMot * gearRatio
    expectedTicksElapsed = cprWheel * revs
    return float(elapsed) / revs
wrn('Expected Seconds/Tick at this speed: %s', getExpectedTickTime(revolutions, trueElapsed))

wrn('All expected Seconds/Tick from LUT (which were actually reproduced experimentally):')
for k, v in revolutionsLUT.items():
    spd, secs = k
    revs = v 
    ticks = getExpectedTickTime(revs, secs)
    wrn('    spd=%s, secs=%s, S/T=%s', getExpectedTickTime(revs, secs))
