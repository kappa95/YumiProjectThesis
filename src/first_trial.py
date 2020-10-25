#!/usr/bin/env python

# import sys
# import copy
import rospy
import moveit_commander
import yumi_utils as yumi
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_srvs.srv import Empty
# import time
from math import pi

# Import the rounded pi.
PI = yumi.PI


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
    yumi.open_grippers(yumi.LEFT)
    yumi.open_grippers(yumi.RIGHT)

    # Reset pose
    rospy.loginfo('Reset pose')
    yumi.reset_pose()
    # rospy.sleep(2.0)

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
    yumi.move_and_grasp(yumi.RIGHT, p_target)
    rospy.sleep(1.0)

    # Opening the gripper
    rospy.loginfo('Opening the gripper')
    yumi.open_grippers(yumi.RIGHT)
    rospy.sleep(0.5)

    # Printing the pose of the right arm
    rospy.loginfo(yumi.get_current_pose(yumi.RIGHT))

    # Randezvous
    # Changing the speed
    fraction = 0.25
    rospy.loginfo('Changing the speed to {}'.format(fraction))
    yumi.change_speed(yumi.RIGHT, fraction)

    # Going to the Target position with z height of 10 cm
    rospy.loginfo('Going to the target position with z at the height of 10cm')
    p_target[1] -= 0.01000
    p_target[2] -= 0.20000
    rospy.sleep(0.1)
    # Go to the position
    yumi.move_and_grasp(yumi.RIGHT, p_target)
    # rospy.sleep(0.5)

    # Going closer
    p_target[2] = z_contact_desk + 0.005
    fraction = 0.50
    rospy.loginfo('Changing the speed to {}'.format(fraction))
    yumi.change_speed(yumi.RIGHT, fraction)

    # Go to the position and close the gripper
    yumi.move_and_grasp(yumi.RIGHT, p_target, 15.0)
    rospy.sleep(0.5)

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
    p_target[2] += 0.20000
    # Increasing the speed from 10% to 25%
    fraction = 1.0
    rospy.loginfo('Changing the speed to {}'.format(fraction))
    yumi.change_speed(yumi.RIGHT, fraction)
    # Go to the position
    yumi.move_and_grasp(yumi.RIGHT, p_target)
    rospy.sleep(1.0)

    # increasing the height and orientate the right arm in horizontal
    p1_R = p_target
    p1_R[0] += 0.150
    p1_R[1] = -z_gripper
    p1_R[2] = 0.300
    p1_R[3] = - pi/2
    p1_R[4] = 0.000
    p1_R[5] = 0.000
    yumi.change_speed(yumi.RIGHT, 0.25)
    yumi.move_and_grasp(yumi.RIGHT, p_target)
    # rospy.sleep(1.0)

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
    yumi.open_grippers(yumi.LEFT)

    # Allineamento del braccio sinistro rispetto al destro
    rospy.loginfo('Allineamento del braccio sinistro su quello destro')
    p1_L[:3] = p1_R[:3]
    p1_L[1] += 0.200 + 2*z_gripper
    p1_L[2] += 0.020
    yumi.change_speed(yumi.LEFT, 0.75)
    yumi.go_to_simple(p1_L[0], p1_L[1], p1_L[2], p1_L[3], p1_L[4], p1_L[5], yumi.LEFT)

    # Avvicinamento del braccio sinistro su quello destro
    rospy.loginfo('Avvicinamento del braccio sinistro su quello destro')
    p1_L[:3] = p1_R[:3]
    p1_L[1] += 2*z_gripper
    p1_L[2] += 0.025000
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
    rospy.sleep(0.2)
    yumi.gripper_effort(yumi.LEFT, 15.0)
    rospy.sleep(0.5)
    yumi.open_grippers(yumi.RIGHT)
    # rospy.sleep(1.0)

    p1_L[1] += 0.10000
    yumi.change_speed(yumi.LEFT, 0.10)
    yumi.go_to_simple(p1_L[0], p1_L[1], p1_L[2], p1_L[3], p1_L[4], p1_L[5], yumi.LEFT)

    # Readings

    rospy.loginfo('reading of the left arm:')
    rospy.loginfo(yumi.get_current_pose(yumi.LEFT))
    rospy.loginfo('reading of the right arm:')
    rospy.loginfo(yumi.get_current_pose(yumi.RIGHT))

    # Prepare a motion with 2 arms together
    rospy.loginfo('Prepare a motion with 2 arms together')
    p2_L = [0.30000, 0.30000, 0.30000, 0, pi, 0]
    p2_R = [0.31500, -0.20200, 0.30000, 0, pi, pi]
    yumi.move_both(p2_L, p2_R)
    yumi.open_grippers(yumi.LEFT)


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
