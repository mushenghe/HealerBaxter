#!/usr/bin/env python

"""
This file contains methods for getting the end effectors to the desired pose

"""

from __future__ import division


import copy
import rospy
import numpy as np

import geometry_msgs.msg
import moveit_msgs.msg

from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_matrix




def made_it(goal, actual, position_tol, orientation_tol):
    """ Test if end effector made it to target pose

    Args:
    goal (Pose): target for robot
    actual (Pose): where robot actually is
    position_tol (float): tolerance for position of EE
    orientation_tol (float): tolerance for orientation of EE

    Returns:
    bool: True for success
    """

    goal = pose_to_list(goal)
    actual = pose_to_list(actual)

    pos_goal = True
    orient_goal = True

    # check if final position is within tolerance
    for index in range(3):
        if abs(actual[index] - goal[index]) > position_tol:
            pos_goal = False

    # check if final orientation is within tolerance
    for index in range(3, 6):
        if abs(actual[index] - goal[index]) > orientation_tol:
            orient_goal = False

    if pos_goal and orient_goal:
        print("Goal Reached")
        return True

    print("Goal Not Reached!")
    return False


def made_it_position(goal, actual, position_tol):
    """ Test if end effector made it to target position

    Args:
    goal (Pose): target for robot
    actual (Pose): where robot actually is
    position_tol (float): tolerance for position of EE

    Returns:
    bool: True for success
    """

    goal = pose_to_list(goal)
    actual = pose_to_list(actual)

    pos_goal = True

    # check if final position is within tolerance
    for index in range(3):
        if abs(actual[index] - goal[index]) > position_tol:
            pos_goal = False

    if pos_goal:
        print("Goal Reached")
        return True

    print("Goal Not Reached!")
    return False






def display_trajectory(plan, robot, display_trajectory_publisher):
    """ Published planned trajectory in rviz

    Args:
        plan: motion plan to goal
        robot: moveit robot commander
        display_trajectory_publisher: rviz publisher
    """

    display_traj = moveit_msgs.msg.DisplayTrajectory()
    display_traj.trajectory_start = robot.get_current_state()
    display_traj.trajectory.append(plan)
    display_trajectory_publisher.publish(display_traj)



def get_transform(pose):
    """ computes the transformation matrix given a pose msg

    Args:
        pose (Pose): pose of a rigid body

    Returns:
        T (np.array): homogenous transformation matrix
    """

    quat_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    homo_rot = quaternion_matrix(quat_list)

    T = np.array([[homo_rot[0][0], homo_rot[0][1], homo_rot[0][2], pose.position.x],
                  [homo_rot[1][0], homo_rot[1][1], homo_rot[1][2], pose.position.y],
                  [homo_rot[2][0], homo_rot[2][1], homo_rot[2][2], pose.position.z],
                  [0,              0,              0,              1]])

    return T



def extract_pose(T):
    """ extracts the quaternion and translation vector from a homogenous
    tranformation matrix

    Args:
        T (np.array): homogenous transformation matrix

    Returns:
        quat (np.array): quaternion represting orientation of rigid body
        trans (np.array): translation vector represting position of rigid body
    """

    # extract euler angles
    rot = np.array([[T[0][0], T[0][1], T[0][2]],
                    [T[1][0], T[1][1], T[1][2]],
                    [T[2][0], T[2][1], T[2][2]]])


    ax, ay, az = euler_from_matrix(rot)
    quat = quaternion_from_euler(ax, ay, az)

    trans = np.array([T[0][3], T[1][3], T[2][3]])

    return quat, trans



def plan_cartesian_path(EE_pose, goal, move_group):
    """ plans a cartesian path using waypoints and allows for re-plannig attempts

    Args:
        EE_pose (Pose): pose of the end effector
        goal (Pose): pose of the goal configuration
        move_group: move group commander
        plan_success (bool): if successfully can cover 80% of the desired path
    """

    print("Terminator Planning Trajectory")

    plan_attempts = 10
    iter = 20
    waypoints = []

    xiter = np.linspace(EE_pose.position.x, goal.position.x, iter)
    yiter = np.linspace(EE_pose.position.y, goal.position.y, iter)
    ziter = np.linspace(EE_pose.position.z, goal.position.z, iter)

    for i in range(iter):
        p = copy.deepcopy(EE_pose)
        p.position.x = xiter[i]
        p.position.y = yiter[i]
        p.position.z = ziter[i]

        # last waypoint use pose of goal
        if i == iter - 1:

            p = copy.deepcopy(goal)
            p.position.x = xiter[i]
            p.position.y = yiter[i]
            p.position.z = ziter[i]

        waypoints.append(p)

    #self.left_group.set_max_velocity_scaling_factor(0.1)
    #self.left_group.set_max_acceleration_scaling_factor(0.1)

    plan_success = True
    fraction = 0.0
    #plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    ctr = 0

    while fraction < 0.8:
        plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        if ctr == plan_attempts:
            plan_success = False
            break

        ctr += 1

    #self.left_group.set_max_velocity_scaling_factor(0.6)
    #self.left_group.set_max_acceleration_scaling_factor(0.6)

    return plan, fraction, plan_success


def move_arm(EE_pose, pose_goal, move_group, robot, display_trajectory_publisher):
    """ Attempts to move the arm in a cartesian path using waypoints if the plan
    to follow tha path fails a target pose goal will be set and the path planning
    component is aborted.

    Args:
        EE_pose (Pose): pose of the end effector
        goal (Pose): pose of the goal configuration
        move_group: move group commander
        robot: moveit robot commander
        display_trajectory_publisher: rviz publisher
    """

    plan, fraction, plan_success = plan_cartesian_path(EE_pose, pose_goal, move_group)

    print("The fraction of path covered")
    print(fraction)

    if plan_success:
        # display plan in rviz
        print("Using cartesian path")
        display_trajectory(plan, robot, display_trajectory_publisher)
        rospy.sleep(2)

        print("============ Press `Enter` to move gripper ============")
        raw_input()

        try:
            # execute path
            move_group.execute(plan, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

            # wait for baxter to get to goal
            rospy.sleep(2)

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.loginfo("Service call failed: %s" % (e,))

    else:
        print("Using pose goal")
        print("============ Press `Enter` to move gripper ============")
        raw_input()

        try:
            move_group.set_pose_target(pose_goal)
            plan = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.loginfo("Service call failed: %s" % (e,))





def arm_init_pose(init_goal, move_group, position_tol, orientation_tol, arm):
    """ places arm is a predefined position during start up

    Args:
        init_goal (Pose): initial pose goal for ende effector
        move_group: move group
        position_tol (float): tolerance for position of EE
        orientation_tol (float): tolerance for orientation of EE
        arm (string): Left or Right arm
    """

    print("Moving" + arm + "arm to initial pose")


    EE_pose = move_group.get_current_pose().pose

    plan, fraction, plan_success = plan_cartesian_path(EE_pose, init_goal, move_group)
    print("Fraction of path covered to initial position")
    print(fraction)

    if plan_success:
        # execute path
        try:
            move_group.execute(plan, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.loginfo("Service call failed: %s" % (e,))

    else:
        try:
            move_group.set_pose_target(init_goal)
            move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.loginfo("Service call failed: %s" % (e,))

    rospy.sleep(2)

    EE_pose = move_group.get_current_pose().pose
    if made_it(init_goal, EE_pose, position_tol, orientation_tol):
        print(arm + "arm is in initial position")

    else:
        print(arm + "arm is Not in initial position")




























#