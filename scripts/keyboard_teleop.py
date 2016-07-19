#!/usr/bin/env python
from time import sleep
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('keyboard_teleop')

pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

while True:
    for control_speed in range(10):
        twist = Twist()
        twist.linear.x = control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        sleep(.1)

