# HealerBaxter
Author: Musheng He
## Project Description:
We live in an era in which communication seems simpler than any times, a friend is only one text away or one video chat away. Although communication may be easier and faster, people still feel lonely and depression rates have largely increased. Inspired by this circumstance, this project is to design a system that would enable the Baxter Robot to detect the negative emotion of its master and play songs on the piano using both hands to help him/her get rid of the bad feelings.

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
``` source ./deep_learning/bin/activate ```
to deactivate :
deactivate 

Use on gpu:  ssh -p 922 129.105.69.167
exit: exit

want to copy files from my computer to GPU:
scp -P 922 test mushenghe@129.105.69.167:~/ 


ctrl-v   go to what ever you want  x
ctrl-v shift i

