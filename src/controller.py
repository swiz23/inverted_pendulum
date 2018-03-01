#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int16
from math import cos,sin,atan,sqrt,pi
import time
from geometry_msgs.msg import Point
from intera_core_msgs.msg import EndpointState
class pen_controller(object):

    def __init__(self):
        rospy.loginfo("Creating pendulum controller class")
        self.ystate = rospy.Publisher("yPoint",Point, queue_size=3)
        self.rolla = rospy.Subscriber("rollAngle",Float32,self.angle)
        self.roll_vel = rospy.Subscriber("ang_vel",Int16,self.speed)
        self.xP = rospy.Subscriber("/robot/limb/right/endpoint_state",EndpointState,self.cart_states)
        self.xbegin = -.208
        self.xPos = 0
        self.xdot = 0
        self.prevroll = 0
        self.command_vel = 0
        self.vel_limit = .5
        self.dt = 0.01
        self.xfinal = -.208
        # self.xaccel = rospy.Subscriber("xAccel",Int16,self.ax)
        # self.yaccel = rospy.Subscriber("yAccel",Int16,self.ay)
        # self.zaccel = rospy.Subscriber("zAccel",Int16,self.az)
        # self.xa = 0
        # self.ya = 0
        # self.za = 10
        self.av = 0
        self.roll = 0
        self.int_timer = rospy.Timer(rospy.Duration(1/1000.), self.yPublish)

    def yPublish(self,tdat):
        a = .1*(self.xPos-self.xbegin)+.5123*self.xdot + 22.6320*self.roll*(pi/180) + 7.2327*self.av*(pi/180)
        # y = cos(rospy.get_time())
        self.command_vel = self.command_vel + a*self.dt
        if (self.command_vel > self.vel_limit):
            self.command_vel = self.vel_limit
        if (self.command_vel < -self.vel_limit):
            self.command_vel = -self.vel_limit
        self.xfinal =self.xfinal + self.command_vel*self.dt
        rospy.loginfo(self.command_vel)
        # if (self.xa is not None and self.ya is not None and self.za is not None):
        #     roll = atan(sqrt(self.xa**2+self.ya**2)/self.za)*180/pi
        # rospy.loginfo(self.roll)
        self.ystate.publish(self.xfinal,self.command_vel,0)
    def angle(self,rollAngle):
        self.prevroll = self.roll
        self.roll = -rollAngle.data
        self.av = (self.roll-self.prevroll)/.01
        # rospy.loginfo(self.av*pi/180)
    # def ax(self,ax_data):
    #     self.xa = ax_data.data
    #     # rospy.loginfo(yacceleration)
    #
    # def ay(self,ay_data):
    #     self.ya = ay_data.data
    #     # rospy.loginfo(yacceleration)
    #
    # def az(self,az_data):
    #     self.za = az_data.data
    #     # rospy.loginfo(yacceleration)

    def speed(self,ang_velocity):
        # self.av = ang_velocity.data;
        i = 0
    def cart_states(self,x_states):
        self.xPos = x_states.pose.position.y
        self.xdot = x_states.twist.linear.y
        # rospy.loginfo(self.xdot)

def main():
    rospy.init_node('controller')

    try:
        PIDcontrol = pen_controller()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
