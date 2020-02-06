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
| week4  1.29 - 2.04  | get a model has 70 % accurency, can detect sad!, piano-playing part setup.  | 1. Apriltag detection failed to detect the tag on the keyboard  -- light condition? 2. failed to do real time detection, can't get the tag detected image -- didn't get the correct image input- remap? // didn't publish to the correct topic? --- the topic is in rostopic list  |  f |f   |
| week5  2.05 - 2.11  |   |   |  f |f   |
| week6  2.12 - 2.18  |   |   |  f |f   |
| week7  2.19 - 2.25  |   |   |  f |f   |
| week8  2.26 - 3.03  |  |  |  f |f   |
| week9  3.04 - 3.10  |   |   |  f |f   |
| week10 3.11 - 3.17  |   |   |   |f   |

/robot/urdf mutable robot state publisher
load up in rviz, make sure matched your robot
2 urdf(on the robot description---internal)(robot state publisher/moveit)


add gripper to robot state publisher----even though it has in the