#!/bin/bash
nmcli connection up Rethink
export ROS_MASTER_URI=http://10.42.0.2:11311
export ROS_IP=10.42.0.1

rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -c head_camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
