#!/usr/bin/env python
import rospy
from time import sleep

from geometry_msgs.msg import Twist
pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

while True:
    for control_speed in range(10):
        twist = Twist()
        twist.linear.x = control_speed;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = control_turn
        pub.publish(twist)
    sleep(.1)
