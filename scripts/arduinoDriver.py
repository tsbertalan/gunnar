#!/usr/bin/env python
from time import sleep

from gunnar.robot import GunnarCommunicator
import rospy
from std_msgs.msg import Float32


class Gunnar(object):
    def __init__(self):
        self._spds = [0, 0]
        self.robotSpeedsStr = ''
        self.communicator = GunnarCommunicator()

    def stop(self):
        self.spds = [0, 0]

    @property
    def spds(self):
        return list(self._spds)

    @spds.setter
    def spds(self, twoList):
        self._spds = twoList
        self.cmdSetSpeeds(twoList[0], twoList[1])
        
    def cmdSetSpeeds(self, a, b):
        self.communicator.speedSet(a, b)
        self.robotSpeedsStr = "(%.1f, %.1f)." % (a, b)

    def spinOnce(self):
        pass
        self.communicator.loopOnce()
        
class VtargetListener(Gunnar):
    
    def __init__(self):
        rospy.init_node('arduino_driver', log_level=rospy.DEBUG)
        super(VtargetListener, self).__init__()
        rospy.loginfo('Begin VtargetListener init.')
        rospy.Subscriber('/lwheel_vtarget', Float32, self.lwheelCallback)
        rospy.Subscriber('/rwheel_vtarget', Float32, self.rwheelCallback)
        rospy.loginfo('Done with VtargetListener init.')
        self.rate = rospy.get_param("~rate", 10)
        print 'Done with VtargetListener init.'
        
    def lwheelCallback(self, data):
        rospy.loginfo('got left wheel data %s' % data.data)
        self.spds = [data.data, self.spds[1]]
        
    def rwheelCallback(self, data):
        rospy.loginfo('got right wheel data %s' % data.data)
        self.spds = [self.spds[0], data.data]
        
    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
    
        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()
            idle.sleep()
        
def main():
    listener = VtargetListener()
    listener.spin()

if __name__ == "__main__":
    main()

