"""
Created on Dec 11, 2016

@author: tsbertalan

twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack


Copyright (C) 2012 Jon Stephan. 
 
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

from __init__ import ROSNode

class TwistToMotors(ROSNode):

    def __init__(self):
    
        self.w = rospy.get_param("~base_width")
    
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
    def main(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    def spinOnce(self):
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        scaleFactor = rospy.get_param('~scaleFactor')
        radFactor = rospy.get_param('~radFactor')
        motor_tune_left = rospy.get_param('~motor_tune_left')
        motor_tune_right = rospy.get_param('~motor_tune_right')
        
        self.left  = scaleFactor * (self.dx - self.dr * self.w / 2 * radFactor) * motor_tune_left
        self.right = scaleFactor * (self.dx + self.dr * self.w / 2 * radFactor) * motor_tune_right
        rospy.logdebug("dx=%s, dr=%s. publishing: (%s, %s)", self.dx, self.dr, self.left, self.right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
            
        self.ticks_since_target += 1

    def twistCallback(self, msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
