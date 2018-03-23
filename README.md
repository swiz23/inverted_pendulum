# Inverted Pendulum

Created by Solomon Wiznitzer for the Master of Science in Robotics program at Northwestern University.

## Description

**Objective:** To balance an inverted pendulum along a straight line using Sawyer, a robot by [Rethink Robotics](http://www.rethinkrobotics.com).

**How It Works:** There were four parts to this project. 
1. Determining how to move Sawyer's seven degree-of-freedom arm such that the end effector followed a straight line path (inverse kinematics)
2. To become familiar with the [CoreUSB](http://wiki.microduinoinc.com/Microduino-Module_CoreUSB), [Motion Sensor](http://wiki.microduinoinc.com/Microduino-Module_Motion), and [Battery Management](http://wiki.microduinoinc.com/Microduino-Module_BM_Li-ion) Microduino modules along with the [BlueSmirf](https://learn.sparkfun.com/tutorials/using-the-bluesmirf) Bluetooth unit such that roll-angle data could be transmitted wirelessly from the top of the pendulum to a laptop.
3. Developing the control algorithm to move Sawyer's end effector and balance the pendulum stably. This involved creating a Mathematica simulation, and using a linearized form of the equations of motion to build a state space model of the system. The model was then used to develop an LQR controller in MATLAB.
4. Solid modeling the [attachment to Sawyer's end effector](media/Solid%20Models/Sawyer_Attachment.stl), the [pendulum holder](media/Solid\Models/Pendulum_holder.stl), and the [Microduino housing](Microduino_house.stl) for the top of the pendulum and then 3D printing them.

**Software Requirements:** Needs ROS, Pyserial, the intera_sdk, the [modern_robotics](https://github.com/NxRLab/ModernRobotics) library, and an Arduino IDE

**Hardware Requirements:** Needs the [Sawyer](http://www.rethinkrobotics.com) robot, the Microduino and Bluetooth modules mentioned above, a small lithium ion battery (100 - 500 mAh), access to a 3D printer, some wires, screws, and a meter long 1/2 inch wooden dowel

**Get Started:** To run, clone the repo, and type `roslaunch inverted_pendulum inv_pen.launch` in the terminal window. Before that step, make sure to connect the Bluetooth module with a laptop by typing `sudo rfcomm bind rfcomm0 [MAC address of Bluetooth module]`. The baud rate of the Bluetooth module should be set to 9600 as well.
[![sawyer_pic](/media/Pictures/vidPic.png)](https://drive.google.com/open?id=1FXJXsdRcDxJXS-Kua1aB4rFsKBY5Y83M)
