# Inverted Pendulum

Created by Solomon Wiznitzer for the Master of Science in Robotics program at Northwestern University.

## Description

**Objective:** To balance an inverted pendulum along a straight line using Sawyer, a robot by [Rethink Robotics](http://www.rethinkrobotics.com).

**How It Works:** There were four parts to this project.
1. Determining how to move Sawyer's seven degree-of-freedom arm such that the end effector followed a straight line path (inverse kinematics)
2. To become familiar with the [CoreUSB](http://wiki.microduinoinc.com/Microduino-Module_CoreUSB), [Motion Sensor](http://wiki.microduinoinc.com/Microduino-Module_Motion), and [Battery Management](http://wiki.microduinoinc.com/Microduino-Module_BM_Li-ion) Microduino modules along with the [BlueSmirf](https://learn.sparkfun.com/tutorials/using-the-bluesmirf) Bluetooth unit such that roll-angle data could be transmitted wirelessly from the top of the pendulum to a laptop.
3. Developing the control algorithm to move Sawyer's end effector and balance the pendulum stably. This involved creating a Mathematica simulation, and using a linearized form of the equations of motion to build a state space model of the system. The model was then used to develop an LQR controller in MATLAB.
4. Solid modeling the [attachment to Sawyer's end effector](media/Solid%20Models/Sawyer_Attachment.stl), the [pendulum holder](media/Solid%20Models/Pendulum_holder.stl), and the [Microduino housing](media/Solid%20Models/Microduino_house.stl) for the top of the pendulum and then 3D printing them.

**Software Requirements:** Needs ROS, Pyserial, the intera_sdk, the [modern_robotics](https://github.com/NxRLab/ModernRobotics) library, and an Arduino IDE

**Hardware Requirements:** Needs the [Sawyer](http://www.rethinkrobotics.com) robot, the Microduino and Bluetooth modules mentioned above, a small lithium ion battery (100 - 500 mAh), access to a 3D printer, some wires, screws, and a meter long 1/2 inch wooden dowel

**Get Started:** To run, clone the repo, and type `roslaunch inverted_pendulum inv_pen.launch` in the terminal window. Before that step, make sure to connect the Bluetooth module with a laptop by typing `sudo rfcomm bind rfcomm0 [MAC address of Bluetooth module]`. The baud rate of the Bluetooth module should be set to 9600 as well.

[![sawyer_pic](/media/Pictures/vidPic.png)](https://drive.google.com/open?id=1FXJXsdRcDxJXS-Kua1aB4rFsKBY5Y83M)
*Sawyer Inverted Pendulum Demo: On left, an Rviz video showing the location of the pendulum tip as it moves in real time. On right, the actual real time footage*

##### [Scripts](src/Python%20Files)
* The [controller.py](src/Python%20Files/controller.py) script launches the node 'controller' that subscribes to the `/rollAngle` topic to collect data as to where the top of the pendulum is (from which angular velocity data is calculated), the `/robot/limb/right/endpoint_state` topic to determine Sawyer's end effector position and velocity, and the `/reset_control` topic which clears the control output to 0 m/s before any demos start. The node calculates the necessary velocity to keep the pendulum balanced using an LQR controller from the four states (end effector position, end effector velocity, pendulum angle, pendulum angular velocity), and publishes the output to the `/yPoint` topic. It also publishes slightly modified rollAngle data based on where the actual 'zero-point' of unstable equilibrium is to the `/newAngle` topic.
* The [ik_horizontal.py](src/Python%20Files/ik_horizontal.py) script launches the node 'ik_horizontal' that subscribes to the `/yPoint` topic mentioned above which inputs the command velocity to the 'x' part of the end-effector twist, along with the `/robot/joint_states` topic which is used to calculate the Jacobian at any given joint configuration. The node waits for the user to press the 'Enter' key at which point it publishes a 'reset' command to the `/reset_control` topic and starts moving the robot arm along the  base frame's 'y-axis'.
* The [rollAngle.py](src/Python%20Files/rollAngle.py) script launches the node 'rollAng' that listens to the `/dev/rfcomm0` serial port (which the bluetooth module is transmitting to) and publishes roll angle data to the `/rollAngle` topic.
* The [pen_marker.py](src/Python%20Files/pen_marker.py) script launches the node 'pen_sim' that subscribes to the `/newAngle` topic. It then publishes the current position of the pendulum with respect to Sawyer's end effector as a marker message (a red sphere) to the `/visualization_marker` topic.

##### Topics

* `/yPoint`: Publishes Sawyer's current end effector velocity relative to the end effector frame, the command velocity from the output of the controller, and Sawyer's current end effector position relative to the base frame

* `/rollAngle`:Publishes the raw angle of pendulum tilt in degrees where 0 corresponds to the unstable equilibrium point

* `/newAngle`: Publishes the same thing as /rollAngle but flips the sign (so positive angle is clockwise instead of left) and adds 0.75 degrees to every angle reading based on where the actual unstable equilibrium point is

* `/reset_control`: Publishes a 'reset' command that is used by the controller to reset the command velocity to 0 m/s before a demo starts

* `/robot/limb/right/endpoint_state`: A Sawyer topic that publishes the current end effector position and velocity (along with other other data that was not used)

* `/robot/joint_states`: A Sawyer topic that publishes the current joint angles used by the 'ik_horizontal' node to find the Jacobian

* `/visualization_marker`: Publishes a red spherical marker to be used by Rviz to display the current position of the pendulum tip relative to Sawyer's end effector.

##### Messages

* *Point*: Part of 'geometry_msgs'. Holds three Float64 numbers in the `/ypoint` topic
* *Float32*: Part of 'std_msgs'. Used in the `/rollAngle` and `/newAngle` topics to hold the angle data
* *String*: Part of 'std_msgs'. Used in the `/reset_control` topic
* *EndpointState*: Part of 'intera_core_msgs' that is used in the `/robot/limb/right/endpoint_state` topic
* *JointState*: Part of 'sensor_msgs' that is used in the `/robot/joint_states` topic to hold angle data
* *Marker*: Part of 'visualization_msgs' used in the `/visualization_marker` topic
* *TransformStamped*: Part of 'geometry_msgs' and is used in finding the transform of Sawyer's right hand with respect to the base

##### [Launch File](/launch)

* [inv_pen.launch](launch/inv_pen.launch): The launch file will enable the robot and start the four nodes mentioned above. It will also launch Rviz with the Sawyer robot model, the TF transforms of the base and right hand frames, and the marker depicting the pendulum tip.


##### [Microduino Code](/src/Microduino%20Code)

* [btConfig.ino](src/Microduino%20Code/btConfig/btConfig.ino): This script was used to configure the Bluetooth module to have a 9600 baud rate via USB. This can also be done without the script wirelessly over Bluetooth.
* [MPU6050_DMP6_NOINT_serial.ino](src/Microduino%20Code/btConfig/MPU6050_DMP_NOINT_serial.ino): This script was based on Jeff Rowberg's code but modified to work with the Microduino. It sends the roll angle data from the Motion Sensor module via Serial1 to the Bluetooth module. The Bluetooth module then transmits it wirelessly to the virtual serial port `/dev/rfcomm0`.
