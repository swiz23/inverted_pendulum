#!/usr/bin/env python
import rospy
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header, Float32
from math import pi,sin,cos

class pen_tracer(object):

    def __init__(self):
        self.marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=10)
        self.rollAng = rospy.Subscriber('newAngle',Float32, self.tracer)
        self.marker_msg = Marker()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rospy_rate = 10
        self.rate = rospy.Rate(self.rospy_rate)
        self.zero_time = rospy.get_time()
        self.count = 0

    def tracer(self,angle):
        try:
            trans = self.tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0))
            # marker x,y,z positions determined from base frame
            self.marker_msg.id = self.count
            self.marker_msg.header.stamp = trans.header.stamp
            self.marker_msg.pose.position.x = trans.transform.translation.x
            self.marker_msg.pose.position.y = trans.transform.translation.y + sin(angle.data*pi/180)
            self.marker_msg.pose.position.z = trans.transform.translation.z + .9144*cos(angle.data*pi/180)
            self.marker_msg.pose.orientation.x = trans.transform.rotation.x
            self.marker_msg.pose.orientation.y = trans.transform.rotation.y
            self.marker_msg.pose.orientation.z = trans.transform.rotation.z
            self.marker_msg.pose.orientation.w = trans.transform.rotation.w
            self.marker_msg.header.frame_id = trans.header.frame_id
            self.marker_msg.scale.x = .1
            self.marker_msg.scale.y = .1
            self.marker_msg.scale.z = .1
            self.marker_msg.type = self.marker_msg.SPHERE
            self.marker_msg.color.a = 1
            self.marker_msg.color.r = 1
            self.marker_msg.color.g = 0
            self.marker_msg.color.b = 0
            self.marker_msg.action = self.marker_msg.ADD
            self.marker_msg.lifetime = rospy.Duration(.05)
            self.marker_pub.publish(self.marker_msg)
            self.count += 1

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

def main():
    rospy.init_node('pen_sim')

    try:
        pt = pen_tracer()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__ == '__main__':
    main()
