#!/usr/bin/env python
import modern_robotics as mr
import sawyer_MR_description as smr
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32,String
from geometry_msgs.msg import Point
import intera_interface
import threading
import matplotlib.pyplot as plt
import time
import csv
from math import pi,radians,sin,cos

class IKhorizontal(object):

    def __init__(self):
        self.angles = [0,0,pi/2,-pi/2,pi/2,pi/2,pi/2+sin(radians(10)) ]
        self.vely = 0
        self.ik_begin = False
        self.goal = mr.FKinBody(smr.M,smr.Blist,self.angles)
        rospy.loginfo(self.goal)
        self.goal[0:3,0:3] = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        rospy.loginfo(self.goal)
        self.js = []
        # self.vel = []
        # self.vellist = []
        # self.veltime = []
        # self.zero_time = 0
        self.limb = intera_interface.Limb("right")
        # self.start_angles = {"right_j0":0, "right_j1":0, "right_j2":0,
        #                     "right_j3": 0, "right_j4":0, "right_j5":0, "right_j6":0}
        self.start_angles = {"right_j0":self.angles[0], "right_j1":self.angles[1], "right_j2":self.angles[2],
                            "right_j3": self.angles[3], "right_j4":self.angles[4], "right_j5":self.angles[5], "right_j6":self.angles[6]}
        self.limb.move_to_joint_positions(self.start_angles)
        self.mutex = threading.Lock()
        self.yp = rospy.Subscriber("yPoint", Point, self.Pnt)
        self.jss = rospy.Subscriber("/robot/joint_states", JointState,self.get_js)
        self.tim = rospy.Timer(rospy.Duration(1/100.), self.step_ik)
        self.key = rospy.Timer(rospy.Duration(1/10.), self.key_press)
        self.status = rospy.Publisher("reset_control",String, queue_size=3)
        self.xdot = 0
        self.count = 0
        self.slow = True
        self.xPos = -.208
    def Pnt(self,points):
        # self.goal[2,3] = points.x
        # self.goal[1,3] = points.x
        self.xdot = points.x
        self.vely = points.y
        self.xPos = points.z
    def get_js(self,joint_st):
        self.js = joint_st.position[1:-1]
        # self.vel = joint_st.velocity[1:-1]

    def key_press(self,td):
        key = raw_input()
        if (key == ""):
            if (self.ik_begin == False):
                self.ik_begin = True
                self.status.publish('reset')
            else:
                self.ik_begin = False
                self.slow = True
                # self.limb.move_to_joint_positions(self.start_angles)
        # elif (key == "r"):
        #     self.ik_begin = False

            # velocity_list = np.array(self.vellist)
            # veltime = np.array(self.veltime)
            # plt.plot(range(0,velocity_list.shape[0]),velocity_list[:,0],label='wx')
            # plt.plot(range(0,velocity_list.shape[0]),velocity_list[:,1],label='wy')
            # plt.plot(range(0,velocity_list.shape[0]),velocity_list[:,2],label='wz')
            # plt.plot(range(0,velocity_list.shape[0]),velocity_list[:,3],label='vx')
            # plt.plot(veltime,velocity_list[:,4],label='vy')
            # plt.plot(range(0,velocity_list.shape[0]),velocity_list[:,5],label='vz')
            # plt.legend()
            # myfile = open('velocity.csv','w')
            # rows = zip(velocity_list[:,4],veltime)
            # with myfile:
            #     writer = csv.writer(myfile)
            #     for row in rows:
            #         writer.writerow(row)

            # plt.show(

    def step_ik(self,tdat):
        # if (self.ik_begin == True):
        with self.mutex:
            g_sd = self.goal
            current_js = self.js
        # Calculate transform from current EE pose to desired EE pose
        err = np.dot(mr.TransInv(mr.FKinBody(smr.M, smr.Blist, current_js)), g_sd)
        # now convert this to a desired twist
        Vb = mr.se3ToVec(mr.MatrixLog6(err))
        if (self.ik_begin == True and self.xPos < .55 and self.xPos > -.7):
            Vb[3] = self.vely
        elif (self.ik_begin == True and self.xPos > .55 or self.xPos < -.7):
            self.ik_begin = False
            self.slow = True
        if (self.vely == 0):
            self.ik_begin = False
            self.slow = True
        if (self.count < pi/2 and self.slow == True):
            Vb[3] = self.xdot*cos(self.count) + Vb[3]*sin(self.count/4)
            self.count = self.count + .015
        elif (self.count >= pi/2 and self.count < 2*pi and self.slow == True):
            Vb[3] =  Vb[3]*sin(self.count/4)
            self.count = self.count + .015
        elif (self.count >= 2*pi and self.slow == True):
            self.count = 0
            self.slow = False



        # calculate body Jacobian at current config
        J_b = mr.JacobianBody(smr.Blist, current_js)
        # now calculate an appropriate joint angle velocity:
        qdot = np.dot(np.linalg.pinv(J_b), Vb)
        # calculate the current ee velocity
        # vtip = np.dot(J_b,self.vel)
        # self.vellist.append(vtip)
        # self.veltime.append(rospy.get_time()-self.zero_time)
        joint_command = {"right_j0":qdot[0], "right_j1":qdot[1], "right_j2":qdot[2],
                            "right_j3": qdot[3], "right_j4":qdot[4], "right_j5":qdot[5], "right_j6":qdot[6]}
        self.limb.set_joint_velocities(joint_command)

def main():
    rospy.init_node('ik_horizontal')

    try:
        ikstraight = IKhorizontal()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
