#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, String, Int16
from math import cos,sin,atan,sqrt,pi
import time
from geometry_msgs.msg import Point
from intera_core_msgs.msg import EndpointState

class pen_controller(object):

    def __init__(self):
        rospy.loginfo("Creating pendulum controller class")
        self.ystate = rospy.Publisher("yPoint",Point, queue_size=3)
        self.rolla = rospy.Subscriber("rollAngle",Float32,self.angle)
        self.adjustedAngle = rospy.Publisher("newAngle",Float32, queue_size=5)
        self.anglePub = rospy.Publisher("angVel",Float32,queue_size=3)
        # self.roll_vel = rospy.Subscriber("ang_vel",Int16,self.speed)
        self.cur_status = rospy.Subscriber("reset_control",String,self.reset)
        self.xP = rospy.Subscriber("/robot/limb/right/endpoint_state",EndpointState,self.cart_states)
        self.xbegin = -0.208
        self.xPos = 0
        self.xdot = 0
        self.prevroll = 0
        self.command_vel = 0
        self.vel_limit = 1
        self.dt = 0.01
        self.av = 0
        self.roll = 0
        self.int_timer = rospy.Timer(rospy.Duration(1/100.), self.yPublish)


    def reset(self,dat):
        if (dat.data == 'reset'):
            self.command_vel = 0

    def yPublish(self,tdat):
        a =.5477*(self.xPos-self.xbegin)+1.5090*self.xdot + 30.1922*self.roll*(pi/180) + 8.3422*self.av*(pi/180)
        self.command_vel = self.command_vel + a*self.dt
        if (self.command_vel > self.vel_limit):
            self.command_vel = self.vel_limit
        if (self.command_vel < -self.vel_limit):
            self.command_vel = -self.vel_limit
        # rospy.loginfo(self.command_vel)

        if (self.roll > 8 or self.roll < -8):
            self.command_vel = 0

        self.ystate.publish(self.xdot,self.command_vel,self.xPos)

    def angle(self,rollAngle):
        self.prevroll = self.roll
        self.roll = -rollAngle.data
        self.roll = self.roll + .75

        i = 0
        average = 0
        while (i<5):
            average = average + (self.roll-self.prevroll)/.01
            i = i + 1

        self.av = average/10
        self.adjustedAngle.publish(self.roll)
        self.anglePub.publish(self.av)

    def speed(self,ang_velocity):
        # plan is to get the gyroscope data straight from the Microduino at a later date
        i = 0
    def cart_states(self,x_states):
        self.xPos = x_states.pose.position.y # wrt base frame
        self.xdot = x_states.twist.linear.x # wrt end effector

def main():
    rospy.init_node('controller')

    try:
        PIDcontrol = pen_controller()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
