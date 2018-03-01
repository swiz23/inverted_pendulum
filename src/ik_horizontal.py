#!/usr/bin/env python
import modern_robotics as mr
import sawyer_MR_description as smr
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from math import pi
import intera_interface
import threading
import matplotlib.pyplot as plt
import time
import csv

class IKhorizontal(object):

    def __init__(self):
        self.angles = [0,0,pi/2,-pi/2,pi/2,pi/2,0]
        self.vely = 0
        self.ik_begin = False
        self.goal = mr.FKinBody(smr.M,smr.Blist,self.angles)
        self.goal[0:3,0:3] = np.eye(3)
        rospy.loginfo(self.goal)
        self.js = []
        # self.vel = []
        # self.vellist = []
        # self.veltime = []
        # self.zero_time = 0
        self.limb = intera_interface.Limb("right")
        self.start_angles = {"right_j0":self.angles[0], "right_j1":self.angles[1], "right_j2":self.angles[2],
                            "right_j3": self.angles[3], "right_j4":self.angles[4], "right_j5":self.angles[5], "right_j6":self.angles[6]}
        self.limb.move_to_joint_positions(self.start_angles)
        self.mutex = threading.Lock()
        self.yp = rospy.Subscriber("yPoint", Point, self.Pnt)
        self.jss = rospy.Subscriber("/robot/joint_states", JointState,self.get_js)
        self.tim = rospy.Timer(rospy.Duration(1/1000.), self.step_ik)
        self.key = rospy.Timer(rospy.Duration(1/10.), self.key_press)

    def Pnt(self,points):
        # self.goal[2,3] = points.x
        # self.goal[1,3] = points.x
        self.vely = points.y
    def get_js(self,joint_st):
        self.js = joint_st.position[1:-1]
        # self.vel = joint_st.velocity[1:-1]

    def key_press(self,td):
        key = raw_input()
        if (key == ""):
            if (self.ik_begin == False):
                self.ik_begin = True
            else:
                self.ik_begin = False
                self.limb.move_to_joint_positions(self.start_angles)
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

            # plt.show()


    def step_ik(self,tdat):
        if (self.ik_begin == True):
            with self.mutex:
                g_sd = self.goal
                current_js = self.js
            # Calculate transform from current EE pose to desired EE pose
            err = np.dot(mr.TransInv(mr.FKinBody(smr.M, smr.Blist, current_js)), g_sd)
            # now convert this to a desired twist
            Vb = mr.se3ToVec(mr.MatrixLog6(err))
            Vb[4] = self.vely
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
