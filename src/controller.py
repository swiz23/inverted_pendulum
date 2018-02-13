#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from math import cos,sin
import time
from geometry_msgs.msg import Point
class pen_controller(object):

    def __init__(self):
        rospy.loginfo("Creating pendulum controller class")
        self.ystate = rospy.Publisher("yPoint",Point, queue_size=3)
        self.int_timer = rospy.Timer(rospy.Duration(1/100.), self.yPublish)

    def yPublish(self,tdat):
        x = .3*sin(rospy.get_time()) + .4
        y = cos(rospy.get_time())
        # y = -.8
        rospy.loginfo(y)
        self.ystate.publish(x,y,0)

def main():
    rospy.init_node('controller')

    try:
        PIDcontrol = pen_controller()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
