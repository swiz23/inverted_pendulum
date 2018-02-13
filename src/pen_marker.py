#!/usr/bin/env python
import rospy
import tf2_ros
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi

def tracer():
    rospy.init_node('pen_sim')
    marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size=10)
    marker_msg = Marker()
    # js_pub = rospy.Publisher('joint_states',JointState,queue_size=10)
    # js = JointState()
    # js.header = Header()
    # js.name = ['right_j0', 'head_pan', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    # js.velocity = []
    # js.effort = []
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    controller_rate = 0.
    rospy_rate = 10
    rate = rospy.Rate(rospy_rate)
    zero_time = rospy.get_time()
    count = 0
    while not rospy.is_shutdown():
        # # rate.sleep()
        # js.header.stamp = rospy.Time.now()
        # # js.header.seq = count
        # js.position = [0,0,0,pi/2,-pi/2,pi/2,pi/2,0]
        # js_pub.publish(js)
        # # rate.sleep()
        try:
            trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        t = rospy.get_time() - zero_time
# marker x,y,z positions determined from base frame
        marker_msg.id = count
        marker_msg.header.stamp = trans.header.stamp
        marker_msg.pose.position.x = trans.transform.translation.x
        marker_msg.pose.position.y = trans.transform.translation.y
        marker_msg.pose.position.z = trans.transform.translation.z + .9144
        marker_msg.pose.orientation.x = trans.transform.rotation.x
        marker_msg.pose.orientation.y = trans.transform.rotation.y
        marker_msg.pose.orientation.z = trans.transform.rotation.z
        marker_msg.pose.orientation.w = trans.transform.rotation.w
        marker_msg.header.frame_id = trans.header.frame_id
        marker_msg.scale.x = .1
        marker_msg.scale.y = .1
        marker_msg.scale.z = .1
        marker_msg.type = marker_msg.SPHERE
        marker_msg.color.a = 1
        marker_msg.color.r = 1
        marker_msg.color.g = 0
        marker_msg.color.b = 0
        marker_msg.action = marker_msg.ADD
        marker_msg.lifetime = rospy.Duration(.05)
        marker_pub.publish(marker_msg)
        count += 1

        # rate.sleep()

if __name__ == '__main__':
    try:
        tracer()
    except rospy.ROSInterruptException:
        pass
