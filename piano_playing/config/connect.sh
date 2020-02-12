#!/bin/bash
nmcli connection up Rethink
export ROS_MASTER_URI=http://10.42.0.2:11311
export ROS_IP=10.42.0.1

rosrun baxter_tools enable_robot.py -e