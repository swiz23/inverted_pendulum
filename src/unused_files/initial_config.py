#!/usr/bin/env python
import rospy
import intera_interface
from math import pi

rospy.init_node("initial_config");

limb = intera_interface.Limb("right");
angles = limb.joint_angles();
angles['right_j0']= 0.0
angles['right_j1']= 0.0
angles['right_j2']=pi/2
angles['right_j3']=-pi/2
angles['right_j4']=pi/2
angles['right_j5']= pi/2
angles['right_j6']=0.0
limb.move_to_joint_positions(angles)
