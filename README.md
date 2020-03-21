# HealerBaxter

This project aims to use deep learning and manipulation platform to train a two-armed robot to play the piano based on the human emotion.

This repository provides Tenserflow code for traning and testing the emotion detection policies with deep learning in real time, as well as the ROS python code for manipulate the robot to play the piano.

<img src="images/emotion_detection.gif" height=300px width="400" align="left"/>
<img src="images/piano_play.gif" height=300px  width="400" align="left"/>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<br/>

## Requirements:
* Linux 18.04
* ROS melodic
* [Kaggle Dataset](https://www.kaggle.com/c/challenges-in-representation-learning-facial-expression-recognition-challenge/data)
## Hardware:
* [Baxter Robot](https://robots.ieee.org/robots/baxter/)
* [Soft roll-up piano](https://www.amazon.com/iMeshbean-Portable-Flexible-Electronic-Keyboard/dp/B07W929C1Y/ref=sr_1_14?keywords=roll+up+piano+61+keys&qid=1577941987&s=toys-and-games&sr=1-14)
* [AprilTags](http://wiki.ros.org/apriltag_ros)
* [AA Batteries](https://www.amazon.com/AmazonBasics-Performance-Alkaline-Batteries-Count/dp/B00MNV8E0C?ref_=s9_apbd_orecs_hd_bw_bQMcmB&pf_rd_r=5RQFAAJHZ8EJ758E09VB&pf_rd_p=a50b8358-02f3-5eb1-a8ca-3ec2c519285c&pf_rd_s=merchandised-search-10&pf_rd_t=BROWSE&pf_rd_i=389577011)


## Technical Goals: 
+ Fallback goal:
    - Assume the person is sad, execute piano playing process
    - Hardcode melody
+ Core goal:
    -Correctly detect person’s facial expression
    -Play great sounds
    -Play chords simultaneously with the melody and play continuously
+ Reach goal:
    -Recognize and distinguish people
    -Baxter have flexibility to play any series of notes

## Learning Objectives:
+ Computer vision
+ Machine learning
+ Deep Learning 
+ Neural networks
+ Face recognition
+ Explore motion planning with modern robotics library

## Tasks:
+ Correctly detect person’s facial expression
    - Choose one of the APIs listed in the Hardware section
    - Use facial expression database and use Opencv and Machine Learning to train the data.
+ Locate the piano and locate  the keys
    - Use AR tag and explore ar_track_alvar
    - Convert the poses in the AR frame to the poses in the world frame.
+ Play great sounds
    - Create a gripper extension that can hit the key without bending or sliding.
    - Set waypoints to ensure the keys are struck directly from above.
+ Play chords simultaneously with the melody and play continuously
    - Use moveit, control both arms
    - Compute all poses by calibrating with the AR tags. As long as the piano is not moved, these will remain constant throughout a series of notes.

## Uncertainties:
+ If the motion planning of Moveit can plan the path and choose the “safe” path
+ If the facial expression detection algorithm works good

## Hardware:
+ Baxter Robot
    - Gripper ( may be customized)
    - Cameras on both arms
+ Roll-up Piano
+ Open source AR Tag/April Tag
+ Facial emotion detection APIs: (or use machine learning algorithms to train / deep learning neural networks)
https://core.ac.uk/download/pdf/160107352.pdf
    - Affectiva
    - Keras
https://github.com/neha01/Realtime-Emotion-Detection
    - Project Oxford by Microsoft https://docs.microsoft.com/en-us/samples/browse/?products=azure
    - Face++
https://www.faceplusplus.com.cn/emotion-recognition/
    - ParallelDocs
https://www.paralleldots.com/facial-emotion
    - NVISO
    https://www.nviso.ai/en
    - Cohn-Kanade AU-Coded Expression Database
http://www.pitt.edu/~emotion/ck-spread.htm

Use tensorflow on my computer:
``` source ./deep_learning/bin/activate 
to deactivate :
deactivate 

Use on gpu:  ssh -p 922 129.105.69.167
exit: exit

want to copy files from my computer to GPU:
scp -P 922 test mushenghe@129.105.69.167:~/ 


ctrl-v   go to what ever you want  x
ctrl-v shift i
```

running in gazebo simulation environment:

run ```roslaunch baxter_gazebo baxter_world.launch``` to start the simulator

run ```roslaunch baxter_moveit_config baxter_grippers.launch``` to bringup robot

run ```rosrun baxter_tools enable_robot.py -e``` to enable the robot:

run ```rosrun baxter_interface joint_trajectory_action_server.py``` to start the joint trajectory controller

run ```rosrun healer initial_pose ``` to launch the initial_pose code

run ```rosrun healer piano_play``` to run the playing piano node