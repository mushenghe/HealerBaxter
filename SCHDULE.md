### Tasks

+  Emotion Detection
    - Face Recognition
    - Emotion Detection
+  Piano Playing
    - Locate the piano and compute the poses of keys
        - explore AR tag, locate the piano
        - compute the offset of each key to the colsest AR tag
        - covert the poses in the AR frame to the poses in the world frame
    - Path planning for both arms
        - learn how to move the gripper apart or closer together    
        - Set waypoint between each movement of notes to create up-down pression motion
        - set all poses continuously

|  | Week Plan  |  Accomplished | Problems&Answers   | Meeting Summary   |
|---|---|---|---|---|
| week1  1.08  - 1.14  |learn emotion detection by deep Learning + what Deep Learning is + what Reinforcement Learning is + learn Tensorflow https://www.coursera.org/specializations/tensorflow-in-practice + learn Keras ||| dont't do both at the same time. training---> piano--->face recognization. Finish deep learning part ASAP. Do tenserflow/pytorch?|
| week2  1.15 - 1.21  |   |   | Trouble using GPU: 1. how to copy files from local to remote(Could not resolve hostname beast: Name or service not known lost connection---ssh.service?) 2.import keras(unable to open X server `' @ error/import.c/ImportImageCommand/358)3. acceess denied (sudo)  | read paper, find(easy to understand&good model)  |
| week3  1.22 - 1.28  |   |   |  f |train a better model, start doing piano-playing part   |
| week4  1.29 - 2.04  | get a model has 70 % accurency, can detect sad!, piano-playing part setup.  | 1. Apriltag detection failed to detect the tag on the keyboard  -- light condition? 2. failed to do real time detection, can't get the tag detected image -- didn't get the correct image input- remap? // didn't publish to the correct topic? --- the topic is in rostopic list  | CONOR/BLOB/LINE DETECTION|f   |
| week5  2.05 - 2.11  |   | able to detect april tag through left wrist camera and publish the pose info. write the function to convert pose to baxter->tag transformation matrix   | baxter throws weird error |KEEP MOVING FORWAED   |
| week6  2.12 - 2.18  |   | add obstacle(the table) to the baxter plan scene, able to move baxter to the initial config smoothly   |  1. python module: can't find the python module I put in the src/my_package dir--->put the module at the same place with the node 2. when trying to move baxter to the first configuration, always extend the arm and stuck.---> use move_to_joint_positions instead of move to the end-effector position. rosrun baxter_examples joint_recorder.py can record the joint position 3. can't move both hands together ---> moveit_commander.MoveGroupCommander("both_arms"), when set pose target, specify the end effector link: group.set_pose_target(left_target_pose, end_effector_link='left_gripper') 4. NO MOTION PLAN FOUND --> try IK? seems the pose I got is not correct?|f   |
| week7  2.19 - 2.25  |   | get the correct pose,able to move both arm to the tag place in simulation  | 1. box not work? 2. tag orientation hard to find. 3. motion plan for controlling both arm at a time is not as good as controlling separate --> cartesian xxxx 4. connection issue (both simulation and real robot)|1. send to matt the dependencies installed 2. transfer learning(after training the model add your) 3. add inflation layer to obstacle 4.  Mutable Robot State Publisher    |
| week8  2.26 - 3.03  | 1. run code on baxter 2.Explore the buildin IK to see if works better than Moveit 3. add inflation layer to obstacle 4.  Mutable Robot State Publisher 5. take a look at transfer learning|  |  f |f   |
| week9  3.04 - 3.10  | make baxter play the piano  |   | 1. tag index keep changing(guess: the first tag been detected is set to index 0 --> not correct)---> after run the initial pose, echo that tag pose topic and modify my code. 2. wait for message(only want the first message) 3. not go straight downward -->have already keep orientation fixed. Not sure why. |f   |
| week10 3.11 - 3.17  |   |   |   |f   |

/robot/urdf mutable robot state publisher
load up in rviz, make sure matched your robot
2 urdf(on the robot description---internal)(robot state publisher/moveit)


add gripper to robot state publisher----even though it has in the