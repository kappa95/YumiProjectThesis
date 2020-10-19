#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import yumi_utils as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
from math import pi
import time


# Import the rounded pi.
PI = yumi.PI


def close_grippers(arm):
    """Closes the grippers.

    Closes the grippers with an effort of 15 and then relaxes the effort to 0.

    :param arm: The side to be closed (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, 15.0)
    yumi.gripper_effort(arm, 0.0)


def open_grippers(arm):
    """Opens the grippers.

    Opens the grippers with an effort of -15 and then relaxes the effort to 0.

    :param arm: The side to be opened (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, -15.0)
    yumi.gripper_effort(arm, 0.0)


def move_and_grasp(arm, pose_ee, grip_effort):
    try:
        yumi.traverse_path([pose_ee], arm, 10)
    except Exception:
        if arm == yumi.LEFT:
            yumi.plan_and_move(
                yumi.group_l, yumi.create_pose_euler(
                    pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4], pose_ee[5]))
        elif arm == yumi.RIGHT:
            yumi.plan_and_move(
                yumi.group_r, yumi.create_pose_euler(
                    pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4], pose_ee[5]))

    if 20 >= grip_effort >= -20:
        yumi.gripper_effort(arm, grip_effort)
    else:
        print("The gripper effort values should be in the range [-20, 20]")


def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """

    rospy.init_node('yumi_trials', anonymous=True)

    # Positions measured
    # x positions
    x_inizio = 0.150  # [m]
    # Length of desk in x direction
    x_desk = 0.360  # [m]
    x_pos = 0.010  # [m]
    x = x_inizio + x_desk + x_pos  # [m]
    # y positions
    y = -0.200
    # z positions
    # Length of the Gripper from datasheet
    z_gripper = 0.136  # [m]
    # Height of the desk from JOGGING
    z_desk = 0.041  # [m]
    z_pos = 0.100  # [m]
    # Contact position measured from JOGGING
    z_contact_desk = 0.171  # [m]
    z = z_desk + z_gripper + z_pos  # [m]
    p1_L = [0.30000, 0.30000, 0.38000, 0, pi, 0]
    # Target of the rubber
    p_target = [0.31500, -0.20200, 0.172, 0, pi, pi]

    # Start by connecting to ROS and MoveIt!
    yumi.init_Moveit()

    # Show the current pose

    # Print current joint angles
    yumi.print_current_joint_states(yumi.RIGHT)
    yumi.print_current_joint_states(yumi.LEFT)

    # Opening the grippers
    open_grippers(yumi.LEFT)
    open_grippers(yumi.RIGHT)

    # Reset pose
    rospy.loginfo('Reset pose')
    yumi.reset_pose()
    yumi.reset_arm(yumi.LEFT)
    yumi.reset_arm(yumi.RIGHT)
    rospy.sleep(2.0)

    rospy.loginfo('La posizione e\':')
    rospy.loginfo(yumi.get_current_pose(yumi.LEFT))

    # Move the left in order to don't busy the space
    rospy.loginfo('Move the left in order to don\'t busy the space')
    yumi.go_to_simple(p1_L[0], p1_L[1], p1_L[2], p1_L[3], p1_L[4], p1_L[5], yumi.LEFT)

    # Picking task
    rospy.loginfo('Picking task')
    # Going to a "close position"
    rospy.loginfo('Going to a close position')
    p_target[2] += 0.300
    print(p_target[2])
    rospy.sleep(0.1)
    # Go to the position
    move_and_grasp(yumi.RIGHT, p_target, 0.0)
    rospy.sleep(1.0)

    # Opening the gripper
    rospy.loginfo('Opening the gripper')
    open_grippers(yumi.RIGHT)
    rospy.sleep(1.0)

    # Printing the pose of the right arm
    rospy.loginfo(yumi.get_current_pose(yumi.RIGHT))

    # Randezvous
    # Changing the speed
    fraction = 0.25
    rospy.loginfo('Changing the speed to {}'.format(fraction))
    yumi.change_speed(yumi.RIGHT, fraction)

    # Going to the Target position with z height of 10 cm
    rospy.loginfo('Going to the target position with z at the height of 10cm')
    p_target[2] -= 0.200
    rospy.sleep(0.1)
    # Go to the position
    move_and_grasp(yumi.RIGHT, p_target, 0)
    rospy.sleep(1.0)

    # Going closer
    p_target[2] = z_contact_desk + 0.005
    fraction = 0.50
    rospy.loginfo('Changing the speed to {}'.format(fraction))
    yumi.change_speed(yumi.RIGHT, fraction)

    # Go to the position and close the gripper
    move_and_grasp(yumi.RIGHT, p_target, 15.0)
    rospy.sleep(1.0)

    rospy.loginfo('La posa nel gripping e\'')
    actual_pose = yumi.get_current_pose(yumi.RIGHT)
    rospy.loginfo(actual_pose)
    rospy.loginfo('Posizione desiderata: {}'.format(p_target))
    errore = [actual_pose.pose.position.x - p_target[0], actual_pose.pose.position.y - p_target[1],
              actual_pose.pose.position.z - p_target[2]]
    rospy.loginfo('L\' errore: {}'.format(errore))
    rospy.sleep(0.1)

    # Raising in z
    rospy.loginfo('Raising in z with gripper closed')
    p_target[2] += 0.200
    # Increasing the speed from 10% to 25%
    fraction = 1.0
    rospy.loginfo('Changing the speed to {}'.format(fraction))
    yumi.change_speed(yumi.RIGHT, fraction)
    # Go to the position
    move_and_grasp(yumi.RIGHT, p_target, 15.0)
    rospy.sleep(1.0)

    # increasing the height and orientate the arm in horizontal
    p1_R = p_target
    p1_R[0] += 0.150
    p1_R[1] = -z_gripper
    p1_R[2] = 0.300
    p1_R[3] = - pi/2
    p1_R[4] = 0.000
    p1_R[5] = 0.000
    yumi.change_speed(yumi.RIGHT, 0.25)
    # move_and_grasp(yumi.RIGHT, p_target, 15.0)
    yumi.go_to_simple(p_target[0], p_target[1], p_target[2],
                      p_target[3], p_target[4], p_target[5], yumi.RIGHT)
    yumi.gripper_effort(yumi.RIGHT, 15.0)
    rospy.sleep(1.0)

    # Printing of the pose
    rospy.loginfo('print the current pose')
    rospy.loginfo(yumi.get_current_pose(yumi.RIGHT))
    rospy.loginfo('print the RIGHT joints values:')
    rospy.loginfo(yumi.get_current_joint_values(yumi.RIGHT))
    rospy.loginfo('print the current pose')
    rospy.loginfo(yumi.get_current_pose(yumi.RIGHT))
    rospy.loginfo('print the LEFT joints values:')
    rospy.loginfo(yumi.get_current_joint_values(yumi.LEFT))

    # Rotazione del braccio sinistro in orizzontale
    rospy.loginfo('Rotazione del braccio sinistro in orizzontale')
    p1_L[3] = - pi/2
    p1_L[4] = pi
    p1_L[5] = pi
    yumi.go_to_simple(p1_L[0], p1_L[1], p1_L[2], p1_L[3], p1_L[4], p1_L[5], yumi.LEFT)
    rospy.sleep(0.5)

    # Open the gripper of the Left Arm
    open_grippers(yumi.LEFT)

    # Allineamento del braccio sinistro su quello destro
    rospy.loginfo('Allineamento del braccio sinistro su quello destro')
    p1_L[:3] = p1_R[:3]
    p1_L[1] += 2*z_gripper
    p1_L[2] += 0.020
    yumi.change_speed(yumi.LEFT, 0.10)
    yumi.go_to_simple(p1_L[0], p1_L[1], p1_L[2], p1_L[3], p1_L[4], p1_L[5], yumi.LEFT)

    # Printing of the pose
    rospy.loginfo('print the current pose: RIGHT')
    rospy.loginfo(yumi.get_current_pose(yumi.RIGHT))
    rospy.loginfo('print the RIGHT joints values:')
    rospy.loginfo(yumi.get_current_joint_values(yumi.RIGHT))
    rospy.loginfo('print the current pose: LEFT')
    rospy.loginfo(yumi.get_current_pose(yumi.LEFT))
    rospy.loginfo('print the LEFT joints values:')
    rospy.loginfo(yumi.get_current_joint_values(yumi.LEFT))

    # Exchange of the rubber
    close_grippers(yumi.LEFT)
    rospy.sleep(0.1)
    open_grippers(yumi.RIGHT)

    p1_L[1] += 0.10000
    yumi.change_speed(yumi.LEFT, 0.10)
    yumi.go_to_simple(p1_L[0], p1_L[1], p1_L[2], p1_L[3], p1_L[4], p1_L[5], yumi.LEFT)

    # Prepare a motion with 2 arms together
    rospy.loginfo('Prepare a motion with 2 arms together')
    p2_L = [0.30000, 0.30000, 0.38000, 0, pi, 0]
    p2_R = [0.31500, -0.20200, 0.20000, 0, pi, pi]
    yumi.move_both(p2_L, p2_R)
    close_grippers(yumi.LEFT)


if __name__ == '__main__':
    try:
        rospy.loginfo('Start of the run')
        run()
        rospy.loginfo('Program finished')
        moveit_commander.roscpp_shutdown()
    except KeyboardInterrupt:
        pass
    except rospy.ROSException:
        print('exception of ROS')
