#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from math import cos
import time

class pen_controller(object):

	def __init__(self):
		rospy.loginfo("Creating pendulum controller class")
		self.ystate = rospy.Publisher("yPoint",Float32, queue_size=3)
		self.int_timer = rospy.Timer(rospy.Duration(1/100.), self.yPublish)

	def yPublish(self,tdat):
		y = cos(rospy.get_time())
		rospy.loginfo(y)
		self.ystate.publish(y)

def main():
    rospy.init_node('controller')

    try:
        PIDcontrol = pen_controller()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
