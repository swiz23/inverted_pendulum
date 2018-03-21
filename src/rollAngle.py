#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import Float32
import time

ser = serial.Serial('/dev/rfcomm0',9600)

def rollPub():
    pub = rospy.Publisher('rollAngle',Float32,queue_size=3)
    rospy.init_node('rollAng')

    while not rospy.is_shutdown():
        try:
            angle = float(ser.readline())
            pub.publish(angle)
        except (serial.SerialException,ValueError):
            rospy.loginfo('Error')
            pass

def main():

    try:
        rollPub()
    except rospy.ROSInterruptException:
        ser.close()

if __name__=='__main__':
    main()
