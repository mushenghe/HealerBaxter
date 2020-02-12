#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np
import yaml
import os
import moveit_commander

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Vector3
from path_planner import PathPlanner
from baxter_interface import Limb
# from intera_interface import Limb
import piano
from baxter_interface import gripper as robot_gripper
# from intera_interface import gripper as robot_gripper
from baxter_interface import RobotEnable
import tf 

import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from intera_interface import HeadDisplay
from PIL import Image as Img 
from PIL import ImageDraw, ImageFont
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker, MarkerArray 
import IPython

cwd = os.getcwd() + "/"

with open(cwd + "cfg.yml", "r") as ymlfile:
    cfg = yaml.load(ymlfile)

listen_topic = cfg["positionTopic"]

waypoint_offset = 0.08
z_offset = 0.099 - .01
electric_z_offset = -0.030 #based on gripper size
calibrate = False 
add_obstacle = False 
five_width = 60
four_width = 15

def play_song(note_pos, note_names):
    """
    Main Script
    """

    planner = PathPlanner("right_arm")

    if add_obstacle:
        table_offset = 0.05
        size = np.array([1, 1, (table_offset - .02) * 2.0])
        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = "base"
        obstacle_pose.pose.position.x = 0.801
        obstacle_pose.pose.position.y = 0.073
        obstacle_pose.pose.position.z = -0.214 - table_offset #remember to calibrate based on ar marker
        planner.add_box_obstacle(size, "table", obstacle_pose)

    new_note_pos = []
    num_waypoints_per_note = 4
    # add waypoints above the note to ensure the note is struck properly
    for note in note_pos:
        waypoint1 = Vector3()
        waypoint1.x = note.x 
        waypoint1.y = note.y 
        waypoint1.z = note.z + waypoint_offset

        waypoint2 = Vector3()
        waypoint2.x = note.x
        waypoint2.y = note.y
        waypoint2.z = note.z + waypoint_offset/2.0

        new_note_pos += [waypoint1, waypoint2, note, waypoint1]

    # Iterate through all note positions for the song
    for i, pos in enumerate(new_note_pos):
        print(i, pos)
        if i % num_waypoints_per_note == 0:
            display_img(note_names[i//num_waypoints_per_note])
        # Loop until that position is reached
        while not rospy.is_shutdown():
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = pos.x
                goal_1.pose.position.y = pos.y
                goal_1.pose.position.z = pos.z

                # #Orientation as a quaternion, facing straight down
                goal_1.pose.orientation.x = 0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0
                goal_1.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_1, list())

                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

def play_chords(note_pos):
    planner = PathPlanner("left_arm")

    new_note_pos = []
    num_waypoints_per_note = 4
    # add waypoints above the note to ensure the note is struck properly
    for note in note_pos:
        waypoint1 = Vector3()
        waypoint1.x = note.x 
        waypoint1.y = note.y 
        waypoint1.z = note.z + waypoint_offset

        waypoint2 = Vector3()
        waypoint2.x = note.x
        waypoint2.y = note.y
        waypoint2.z = note.z + waypoint_offset/2.0

        new_note_pos += [waypoint1, waypoint2, note, waypoint1]

    for i, pos in enumerate(new_note_pos):
        print(i, pos)
        # Loop until that position is reached
        while not rospy.is_shutdown():
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = pos.x
                goal_1.pose.position.y = pos.y
                goal_1.pose.position.z = pos.z
                # #Orientation as a quaternion, facing straight down
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_1, list())

                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break
def create_path_with_waypoints(notes_only):
    path_with_waypoints = []
    # add waypoints above the note to ensure the note is struck properly
    for note in notes_only:
        waypoint1 = Vector3()
        waypoint1.x = note.x 
        waypoint1.y = note.y 
        waypoint1.z = note.z + waypoint_offset

        waypoint2 = Vector3()
        waypoint2.x = note.x
        waypoint2.y = note.y
        waypoint2.z = note.z + waypoint_offset/2.0

        path_with_waypoints += [waypoint1, waypoint2, note, waypoint1]

    return path_with_waypoints

def play_melody_and_chords(melody_note_pos, chords_note_pos, chord_lengths, left):
    """ Both args are lists of the same len """

    # If the node is shutdown, call this function    
    # rospy.on_shutdown(shutdown)

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a move group
    two_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    # Set the maximum time MoveIt will try to plan before giving up
    two_arms_group.set_planning_time(5)

    # Set the bounds of the workspace
    two_arms_group.set_workspace([-2, -2, -2, 2, 2, 2])

    # Sleep for a bit to ensure that all inititialization has finished
    rospy.sleep(0.5)   

    ### Plan ###
    # Create path with waypoints
    melody_new_note_pos = create_path_with_waypoints(melody_note_pos)
    chord_new_note_pos = create_path_with_waypoints(chords_note_pos)

    for i, (melody_note, chord) in enumerate(zip(melody_new_note_pos, chord_new_note_pos)):
        while not rospy.is_shutdown():
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = melody_note.x
                goal_1.pose.position.y = melody_note.y
                goal_1.pose.position.z = melody_note.z
                # #Orientation as a quaternion, facing straight down
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0
                two_arms_group.set_pose_target(goal_1, "right_gripper")

                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = chord.x
                goal_2.pose.position.y = chord.y
                goal_2.pose.position.z = chord.z
                # #Orientation as a quaternion, facing straight down
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0
                two_arms_group.set_pose_target(goal_2, "left_gripper")
                print(melody_note, chord)
                print(goal_1, goal_2)
                #two_arms_group.set_start_state_to_current_state()
                plan = two_arms_group.plan()

                if i % 4 == 0:
                    chord_len = chord_lengths[i//4]                
                    if chord_len == 5:
                        left.command_position(five_width)
                    else:
                        left.command_position(four_width)

                if not two_arms_group.execute(plan, True):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

                

# def shutdown(group):
#         """
#         Code to run on shutdown. This is good practice for safety

#         Currently deletes the object's MoveGroup, so that further commands will do nothing
#         """
#         group = None
#         rospy.loginfo("Stopping Path Planner")


def create_note(x, y, z):
    note = Vector3()
    note.x = x
    note.y = y
    note.z = z + z_offset
    return note


def get_transform_g(frame1, frame2):
    listener = tf.TransformListener()

    listener.waitForTransform(frame1, frame2, rospy.Time(), rospy.Duration(15)) # wait for tf to be generated
    (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))

    R = tf.transformations.quaternion_matrix(rot)
    euler = tf.transformations.euler_from_quaternion(rot) #roll, pitch, yaw

    R[0][3] = trans[0]
    R[1][3] = trans[1]
    R[2][3] = trans[2]

    return R


def get_ar_transforms(ar_marker_keys):
    #Use ar_track_alvar to find postion of tag centers
    ar_transforms = {}
    for ar_marker_key in ar_marker_keys:
        g_base_ar = get_transform_g('/reference/base', '/ar_marker_' + str(ar_marker_key)) 
        R_sum = g_base_ar[0:3][0:3]
        euler = tf.transformations.euler_from_matrix(R_sum, axes='sxyz')
        ar_transforms[ar_marker_key] = (g_base_ar, euler)
    return ar_transforms


def convert_song_to_coords(str_notes, notes_to_coords_map):
    vector_coords = []
    for note in str_notes:
        coord = notes_to_coords_map.get(note, None) #TODO CHANGE THIS
        vector_coord = Vector3()
        vector_coord.x = coord[0]
        vector_coord.y = coord[1]
        vector_coord.z = coord[2]
        vector_coords.append(vector_coord)

    return vector_coords

def make_key_coords_reversed(ar_tags_to_position):
    # Maps keys (strings) to positions in the ar tag frame
    offset_y_black = 5.0

    white_center_to_white_center = 2.2
    white_center_to_black_center = 0.9
    black_center_to_black_center = 2.8

    note_names = ["D", "E", "F", "G", "A", "B", "C"]
    sharp_names = ["C", "D", "F", "G", "A"]

    key_coords_dict = {}

    # WHITE KEYS
    key_coords_dict["C0"] = np.array([0, 0])
    for i in range(21):
        note_name = note_names[i % 7] + str((i+1)//7)

        key_coords_dict[note_name] = (i+1)*np.array([white_center_to_white_center, 0])

    # BLACK KEYS
    for j in range(15):
        octave_num = j//5
        note_name = sharp_names[j%5] + str(octave_num) + "#"
        black_key_offset = np.array([white_center_to_black_center, offset_y_black])
        if (j % 5) <= 1:
            sharp_ind = j % 5
            key_coords_dict[note_name] = key_coords_dict["C" + str(octave_num)] + np.array([black_center_to_black_center * sharp_ind, 0]) + black_key_offset
        else:
            sharp_ind = (j - 2) % 5
            key_coords_dict[note_name] = key_coords_dict["F" + str(octave_num)] + np.array([black_center_to_black_center * sharp_ind, 0]) + black_key_offset

    ar_tags_to_key_coords_dicts = {}
    for ar_tag in ar_tags_to_position:
        curr_key_coords_dict = {}
        curr_center_key = ar_tags_to_position[ar_tag]
        for key in key_coords_dict:
            curr_key_coords_dict[key] = key_coords_dict[key] - key_coords_dict[curr_center_key]
        ar_tags_to_key_coords_dicts[ar_tag] = curr_key_coords_dict
    return ar_tags_to_key_coords_dicts

def display_img(note_name):
    fname = "note_imgs/" + note_name + ".png"

    img = cv2.imread("note_imgs/" + note_name + ".png")
    msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)

    pub.publish(msg)
    rospy.sleep(0.1)

    # head_display = HeadDisplay()
    # head_display.display_image(fname)

def save_note_image(note_name):
    img = Img.new('RGB', (1000, 600), color = (255, 255, 255))
 
    d = ImageDraw.Draw(img)
    fnt = ImageFont.truetype('Pillow/Tests/fonts/FreeMono.ttf', 250)
    d.text((400,200), note_name, font=fnt, fill=(0,0,0))
     
    img.save("note_imgs/" + note_name + ".png")

def make_note_images():
    for note_name in ["C", "D", "E", "F", "G", "A", "B"]:
        for i in range(3):
            save_note_image(note_name + str(i))
    save_note_image("C3")

    for sharp_name in ["A", "G", "F", "D", "C"]:
        for i in range(3):
            save_note_image(sharp_name + str(i) + "#")

    rospy.sleep(1)

def graph_markers(key_poses, ar_num):
    markerPub = rospy.Publisher('robotMarker' + str(ar_num), MarkerArray, queue_size=10)
    rate = rospy.Rate(1)

    robotMarkersArray = MarkerArray()
    robotMarkers = [] 
    ind = 0
    for k in key_poses:
        robotMarker = Marker()
        robotMarker.header.frame_id = "/base"
        robotMarker.header.stamp = rospy.get_rostime()
        robotMarker.ns = "robot"
        robotMarker.id = ind
        robotMarker.type = 2 # sphere
        robotMarker.action = 0
        robotMarker.pose.position.x = key_poses[k].x 
        robotMarker.pose.position.y = key_poses[k].y 
        robotMarker.pose.position.z = key_poses[k].z 
        robotMarker.pose.orientation.x = 0
        robotMarker.pose.orientation.y = 0
        robotMarker.pose.orientation.z = 0
        robotMarker.pose.orientation.w = 1.0
        robotMarker.scale.x = 0.01
        robotMarker.scale.y = 0.01
        robotMarker.scale.z = 0.01

        robotMarker.color.r = 0.0
        robotMarker.color.g = 1.0
        robotMarker.color.b = 0.0
        robotMarker.color.a = 1.0
        robotMarker.lifetime = rospy.Duration(0)

        robotMarkers.append(robotMarker) 
        ind += 1
    robotMarkersArray.markers = robotMarkers 

    while not rospy.is_shutdown():
        markerPub.publish(robotMarkersArray)
        rate.sleep()
def avg_vector3(vector1, vector2):
    x = vector1.x + vector2.x
    y = vector1.y + vector2.y
    z = vector1.z + vector2.z
    print("total z", z)
    avgVector = Vector3()
    avgVector.x = x/2.0
    avgVector.y = y/2.0
    avgVector.z = z/2.0 - z_offset
    print("avg Z", avgVector.z)
    return avgVector

# def get_vector_coord_for_note(note_name, ar_to_key_poses):
#     if #A1

#return true if note1 is above or equal to note2
def is_above(note1, note2):
    oct1, oct2 = note1[1], note2[1]
    if oct1 > oct2:
        return True
    elif oct1 < oct2:
        return False
    else:
        note_ordering = ['C', 'D', 'E', 'F', 'G', 'A', 'B']
        ind1, ind2 = note_ordering.index(note1[0]), note_ordering.index(note2[0])
        return ind1 >= ind2

def get_vector_coord_for_note(note_name, ar_to_key_poses, ar_tags_to_positions):
    curr_reference_note = None
    curr_ar_tag = None
    for ar_tag, reference_note in ar_tags_to_positions.items():
        if is_above(note_name, reference_note):
            if curr_reference_note is None or is_above(reference_note, curr_reference_note):
                curr_reference_note = reference_note
                curr_ar_tag = ar_tag
    return ar_to_key_poses[curr_ar_tag][note_name]

#WARNING!!!!!!!!!!!!!!!: must run following command before this script
# ./run_demo.sh
if __name__ == '__main__':
    rospy.init_node('moveit_node')
    # make_note_images()

    ar_tags_to_positions = {4: "C0", 1: "C2"}
    ar_to_transforms = get_ar_transforms(ar_tags_to_positions.keys())

    ar_to_key_coords_dict = make_key_coords_reversed(ar_tags_to_positions)

    ar_to_key_poses = {}
    for ar_tag_number in ar_to_transforms:
        key_poses_dict = {}
        for k in ar_to_key_coords_dict[ar_tag_number]:
            g_base_ar, euler = ar_to_transforms[ar_tag_number]
            x_cm, y_cm = ar_to_key_coords_dict[ar_tag_number][k]

            key_pose_in_ar = np.array([[x_cm/100.0, y_cm/100.0, 0, 1]]).T
            key_pose_in_base = g_base_ar.dot(key_pose_in_ar)
            key_pose_in_base_formatted = create_note(key_pose_in_base[0][0], key_pose_in_base[1][0], g_base_ar[2][3])
            key_poses_dict[k] = key_pose_in_base_formatted
        ar_to_key_poses[ar_tag_number] = key_poses_dict

    note_names = ['C2', 'G2', 'A2', 'G2', 'F2', 'E2', 'D2', 'C2', 'G2', 'F2', 'E2', 'D2', 'G2', 'F2', 'E2', 'D2']


    vector_coords = [get_vector_coord_for_note(n, ar_to_key_poses, ar_tags_to_positions) for n in note_names]

    
    left = robot_gripper.Gripper('left') #make sure to change this
    rospy.sleep(1)
    left.calibrate()

    left.command_position(60) #originally 67

    #currently only for size 5 chords 
    # Create chords to accompany "Twinkle Twinkle" melody
    chords = []
    for melody_note in note_names:
            if melody_note[0] == "C" or melody_note[0] == "E" or melody_note[0] == "G":
                chords.append(['C0', 'G0'])
            elif melody_note[0] == "F" or melody_note[0] == "A":
                chords.append(['F0', 'C1'])
            elif melody_note[0] == "D":
                chords.append(['F0', 'B0'])

    # chords = [('C0', 'G0')]*len(note_names)
    chord_lengths = []
    for c in chords:
        if c == ['C0', 'G0'] or c == ['F0', 'C1']:
            chord_lengths.append(5)
        else:
            chord_lengths.append(4)
    chord_coords = []
    for chord in chords:
        note1, note2 = chord 
        print("created note")
        print(get_vector_coord_for_note(note1, ar_to_key_poses, ar_tags_to_positions))
        print(get_vector_coord_for_note(note2, ar_to_key_poses, ar_tags_to_positions))
        avg_vector = avg_vector3(get_vector_coord_for_note(note1, ar_to_key_poses, ar_tags_to_positions), get_vector_coord_for_note(note2, ar_to_key_poses, ar_tags_to_positions))
        chord_coords.append(avg_vector)
        #vector_coords.append(key_poses_dict[note1])

    #lay_chords(vector_coords)
    print("we good")

    play_melody_and_chords(vector_coords, chord_coords, chord_lengths, left)

"""
Remember to:
1) Update physical side length of AR tags in launch
2) Run joint traj action server, moveit node, artrackalvar launch file
3) Remember in rviz to set fixed_frame as base
4) back of piano is about 15 inches from bolt at the front of asimov's base
"""

# #Full Gripper position (100)
# # 4-chord (10)
# # 5-chord (70)
# # 6-chord ()