# tobe_gait

TOBE is an 18-DOF position controlled robot with eighteen Dynamixel AX-12+ motors. This repository is based on a modified version of the central pattern generator (CPG)-based gait described in chapter 6 of Missura's PhD thesis "Analytic and Learned Footstep Control for Robust Bipedal Walking". The modifications I made to the BIOLOID robot (which resulted in TOBE's different arm and leg configurations) and to the CPG gait (which resulted in gait produced by the code of this repo) are outlined in chapter 3 of my PhD thesis "Model-Free Control Methods for Gait and Push Recovery in Bipedal Humanoid Robots". 

There are five files in this repository that accomplish the gait motions of TOBE:
1. tobecontrol.yaml -- Find here: catkin_ws/src/tobe3_gait/config. This file sets up joint position controllers for the 18 joints.

2. realtobe_CPG_walk.launch -- Find here: catkin_ws/src/tobe3_gait/launch. This file is accessed with 'roslaunch', to execute a ROS node for producing gait motions for the robot.

3. tobe_walk.py -- Find here: catkin_ws/src/tobe3_gait/scripts. This file defines the ROS node to be run for executing gait motions. NOTE: Make sure that this file is executable (right-click on the file, select Properties, click on the Permissions tab, and make sure that the box 'Allow executing file as program' is checked).

4. tobe.py -- Find here: catkin_ws/src/tobe3_gait/src/tobe3_gait. This file sets up the USB connection to the robot, and turns on the motors. It also defines several functions for sending commands to the motors, publishing joint angles and velocities to corresponding ROS topics, and converting between 10-bit joint commands and joint angles (in radians). 

5. walker.py -- Find here: catkin_ws/src/tobe3_gait/src/tobe3_gait. This file contains the CPG gait controller. 


ROS-related dependencies:
This has been written with ROS1 Noetic, so it may not be compatible with other versions. To install ROS Noetic, follow the instructions at wiki.ros.org/noetic/installation/Ubuntu. 

If you have not already modified your BASHRC script file, or don't know what it is: 
The file ~/.bashrc is another useful file, not a part of this repository, that should be modified as follows:
1. Add the line: source ~/tobe_gait/devel/setup.bash (if you haven't already).
2. Set the ROS_IP by adding (or changing) the line: export ROS_IP=10.4.128.228 (use YOUR IP address instead of 10.4.128.228).
3. Set the ROS_MASTER_URI by adding (or changing) the line: export ROS_MASTER_URI=http://10.4.128.228:11311 (change 10.4.128.228 to YOUR IP address, and do not modify the :11311 suffix).
4. Add the src folder containing the tobe.py and walker.py files to your Python path by adding: export PYTHONPATH=~/tobe_gait/catkin_ws/src/tobe3_gait/src:$PYTHONPATH
5. Add the larger src folder to your ROS package path by adding: export ROS_PACKAGE_PATH=~/tobe_gait/catkin_ws/src:$ROS_PACKAGE_PATH

The ROS node will likely not launch without ROS_IP and/or ROS_MASTER_URI properly set to your IP address, since roscore requires this. 


Other dependencies:
Install the Python library dynamixel-controller using pip. In a terminal, use the following command: pip install dynamixel-controller
The tobe.py file relies on this library to connect to the motors. 

Video tutorial:
A video tutorial of how to use this code is available at https://youtu.be/y4QyoEVgmbI
