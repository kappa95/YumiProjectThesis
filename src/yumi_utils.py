#!/usr/bin/env python

import sys
import copy
import rospy
import tf.transformations
import moveit_commander
from moveit_commander import *
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from yumi_hw.srv import *
from rospy_message_converter import message_converter


RIGHT = 1  # :ID of the right arm
LEFT = 2  # :ID of the left arm
BOTH = 3  # :ID of both_arms
PI = 3.1415926  # :Value of PI

table_height = 0.025  # :The height of the upper surface of the table
table_width = 0.400
# planner = "RRTstarkConfigDefault"  # Custom planner
# planner = "ESTkConfigDefault"  # Type of the planner
planner = "RRTConnectConfigDefault"  # Custom planner
planner_L = planner

planning_attempts = 100  # planning attempts
planning_time = 50  # [s] Planning time for computation

global group_l  # :The move group for the left arm
global group_r  # :The move group for the right arm
global group_both  # :The move group for using both arms at once
global robot  # :The RobotCommander() from MoveIt!
global scene  # :The PlanningSceneInterface from MoveIt!


# Initializes the package to interface with MoveIt!
def init_Moveit():
    """Initializes the connection to MoveIt!

    Initializes all the objects related to MoveIt! functions. Also adds in the
    table to the MoveIt! planning scene.

    :returns: Nothing
    :rtype: None
    """

    global group_l
    global group_r
    global group_both
    global robot
    global scene
    print("####################################     Start Initialization     ####################################")
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    mpr = moveit_msgs.msg.MotionPlanRequest()
    rospy.sleep(1.0)

    # clean the scene
    rospy.loginfo('Cleaning of the objects in the scene')
    rospy.loginfo('Cleaning of the table')
    scene.remove_world_object("table")
    rospy.sleep(0.5)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    # mpr.max_velocity_scaling_factor = 1.0

    # add the table table
    rospy.loginfo('Adding the table object')
    table_pose = PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    table_pose.pose.position.x = 0.150 + table_width / 2
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = table_height / 2
    scene.add_box("table", table_pose, size=(table_width, 1.2, table_height))

    # Left arm
    group_l = moveit_commander.MoveGroupCommander("left_arm")
    # Type of planner
    group_l.set_planner_id(planner_L)
    group_l.set_pose_reference_frame("yumi_body")

    # Replanning
    group_l.allow_replanning(True)
    group_l.set_goal_position_tolerance(0.005)
    group_l.set_goal_orientation_tolerance(0.005)
    group_l.set_num_planning_attempts(planning_attempts)
    group_l.set_planning_time(planning_time)
    print('For the Left arm the end effector link is')
    print(group_l.get_end_effector_link())
    print(group_l.get_planning_frame())

    # Right arm
    group_r = moveit_commander.MoveGroupCommander("right_arm")
    # Type of planner
    group_r.set_planner_id(planner)
    group_r.set_pose_reference_frame("yumi_body")
    # Replanning
    group_r.allow_replanning(True)
    group_r.set_goal_position_tolerance(0.005)
    group_r.set_goal_orientation_tolerance(0.005)
    group_r.set_num_planning_attempts(planning_attempts)
    group_r.set_planning_time(planning_time)
    print('For the Right arm the end effector link is')
    print(group_r.get_end_effector_link())
    print(group_l.get_planning_frame())

    # Both arms
    group_both = moveit_commander.MoveGroupCommander("both_arms")
    # Type of planner
    group_both.set_planner_id(planner_L)

    # Pose reference frame is the yumi_body
    group_both.set_pose_reference_frame("yumi_body")
    # Replanning
    group_both.allow_replanning(True)
    group_both.set_goal_position_tolerance(0.005)
    group_both.set_goal_orientation_tolerance(0.005)
    group_both.set_num_planning_attempts(planning_attempts)
    group_both.set_planning_time(planning_time)

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    rospy.sleep(3)
    print("####################################     Finished Initialization     ####################################")

    sys.stdout.write('\nYuMi MoveIt! demo initialized!\n\n\n')


def print_current_pose(arm):
    """Prints the current pose

    Prints the current pose of the selected arm into the terminal

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    pose = get_current_pose(arm)
    rpy = get_current_rpy(arm)

    if arm == LEFT:
        print("Left arm pose and rpy:")
    if arm == RIGHT:
        print("Right arm pose and rpy:")

    print(pose)
    print(rpy)


def get_current_pose(arm):
    """Gets the current pose

    Return the current pose of the selected arm

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Pose
    :rtype: PoseStamped
    """
    global group_l
    global group_r
    if arm == LEFT:
        return group_l.get_current_pose()
    if arm == RIGHT:
        return group_r.get_current_pose()


def change_speed(arm, speed_factor):
    """
    Change the speed of the arm
    :param arm: The arm could be LEFT or RIGHT
    :type arm: int
    :param speed_factor: Fraction of the maximum speed
    :type speed_factor: float
    :return: Nothing
    """

    global group_l
    global group_r
    if arm == LEFT:
        group_l.set_max_velocity_scaling_factor(speed_factor)
    elif arm == RIGHT:
        group_r.set_max_velocity_scaling_factor(speed_factor)
    else:
        print("Settato braccio sbagliato")


def get_current_rpy(arm):
    """Gets the current orientation

    Returns the current orientation of the selected arm as Euler angles

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Orientation
    :rtype: Orientation
    """
    global group_l
    global group_r
    if arm == LEFT:
        return group_l.get_current_rpy()
    if arm == RIGHT:
        return group_r.get_current_rpy()


def get_current_joint_values(sel):
    """Gets the current joint values

    Returns the current position of all joints in the selected arm. From J1-J7

    :param sel: The arm to display the pose of (RIGHT or LEFT)
    :type sel: int
    :returns: Joint values
    :rtype: float[]
    """
    global group_l
    global group_r

    if sel == RIGHT:
        return group_r.get_current_joint_values()
    if sel == LEFT:
        return group_l.get_current_joint_values()


def print_current_joint_states(arm):
    """Prints the current joint values

    Prints the current joint values of the selected arm into the terminal

    :param arm: The arm to display the pose of (RIGHT or LEFT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """

    if arm == LEFT:
        print("Left arm joint states:")
    if arm == RIGHT:
        print("Right arm joint states:")

    print(get_current_joint_values(arm))


# Create a pose object from position and orientation (euler angles)
def create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad):
    """Creates a pose using euler angles

    Creates a pose for use with MoveIt! using XYZ coordinates and RPY
    orientation in radians

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param roll_rad: The roll angle for the pose
    :param pitch_rad: The pitch angle for the pose
    :param yaw_rad: The yaw angle for the pose
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type roll_rad: float
    :type pitch_rad: float
    :type yaw_rad: float
    :returns: Pose
    :rtype: PoseStamped
    """
    quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    return create_pose(x_p, y_p, z_p, quaternion[0], quaternion[1], quaternion[2], quaternion[3])


# Create a pose object from position and orientation (quaternion)
def create_pose(x_p, y_p, z_p, x_o, y_o, z_o, w_o):
    """Creates a pose using quaternions

    Creates a pose for use with MoveIt! using XYZ coordinates and XYZW
    quaternion values

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param x_o: The X-value for the orientation
    :param y_o: The Y-value for the orientation
    :param z_o: The Z-value for the orientation
    :param w_o: The W-value for the orientation
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type x_o: float
    :type y_o: float
    :type z_o: float
    :type w_o: float
    :returns: Pose
    :rtype: PoseStamped
    """
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x_p
    pose_target.position.y = y_p
    pose_target.position.z = z_p
    pose_target.orientation.x = x_o
    pose_target.orientation.y = y_o
    pose_target.orientation.z = z_o
    pose_target.orientation.w = w_o
    return pose_target


# Set the gripper to an effort value
def gripper_effort(gripper_id, effort):
    """Set gripper effort

    Sends an effort command to the selected gripper. Should be in the range of
    -20.0 (fully open) to 20.0 (fully closed)

    :param gripper_id: The ID of the selected gripper (LEFT or RIGHT)
    :param effort: The effort value for the gripper (-20.0 to 20.0)
    :type gripper_id: int
    :type effort: float
    :returns: Nothing
    :rtype: None
    """
    rospy.loginfo("Setting gripper " + str(gripper_id) + " to " + str(effort))
    rospy.loginfo('Setting gripper effort to ' + str(effort) + ' for arm ' + str(gripper_id))

    if gripper_id == RIGHT:
        pubname = '/yumi/gripper_r_effort_cmd'

    if gripper_id == LEFT:
        pubname = '/yumi/gripper_l_effort_cmd'

    pub = rospy.Publisher(pubname, std_msgs.msg.Float64, queue_size=10, latch=True)
    pub.publish(std_msgs.msg.Float64(effort))
    rospy.sleep(1.0)


# Wrapper for plan_and_move, just position, orientation and arm
def go_to_simple(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad, arm):
    """Set effector position

    Sends a command to MoveIt! to move to the selected position, in any way
    it sees fit.

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param roll_rad: The roll angle for the pose
    :param pitch_rad: The pitch angle for the pose
    :param yaw_rad: The yaw angle for the pose
    :param arm: The arm to execute the movement
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type roll_rad: float
    :type pitch_rad: float
    :type yaw_rad: float
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    if arm == LEFT:
        plan_and_move(group_l, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))
    elif arm == RIGHT:
        plan_and_move(group_r, create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad))


# Make a plan and move within the joint space
def go_to_joints(positions, arm):
    """Set joint values

    Moves the selected arm to make the joint positions match the given values

    :param positions: The desired joint values [j1-j7] (or [j1l-j7l,j1r-j7r] for both arms at once)
    :param arm: The selected arm (LEFT, RIGHT or BOTH)
    :type positions: float[]
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r
    global group_both

    cur_arm = group_l
    if arm == RIGHT:
        cur_arm = group_r
    elif arm == BOTH:
        cur_arm = group_both

    if arm == BOTH:
        cur_arm.set_joint_value_target(positions[0] + positions[1])
    else:
        cur_arm.set_joint_value_target(positions)
    cur_arm.plan()
    cur_arm.go(wait=True)
    rospy.sleep(3)


def plan_path(points, arm, planning_tries=500):
    global robot
    global group_l
    global group_r

    if arm == LEFT:
        armname = 'left'
        cur_arm = group_l
    if arm == RIGHT:
        armname = 'right'
        cur_arm = group_r

    rospy.loginfo('Moving ' + armname + ' through path: ')
    rospy.loginfo(points)

    waypoints = []
    # waypoints.append(cur_arm.get_current_pose().pose)
    for point in points:
        wpose = create_pose_euler(point[0], point[1], point[2], point[3], point[4], point[5])
        waypoints.append(copy.deepcopy(wpose))

    # print waypoints
    cur_arm.set_start_state_to_current_state()

    attempts = 0
    fraction = 0.0

    while fraction < 1.0 and attempts < planning_tries:
        (plan, fraction) = cur_arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
        attempts += 1
        rospy.loginfo('attempts: ' + str(attempts) + ', fraction: ' + str(fraction))
        if fraction == 1.0:
            plan = cur_arm.retime_trajectory(robot.get_current_state(), plan, 1.0)
            return plan
            # r = cur_arm.execute(plan)

    if fraction < 1.0:
        rospy.logerr('Only managed to calculate ' + str(fraction * 100) + '% of the path!')
        raise Exception('Could not calculate full path, exiting')

    return None


# Plan a cartesian path through the given waypoints and execute
def traverse_path(points, arm, planning_tries=500):
    """Commands an end-effector to traverse a path

    Creates a path between the given waypoints, interpolating points spaced
    by 1cm. Then tries to plan a trajectory through those points, until it
    succeeds and executes or if it fails to find a path for 500 tries it throws
    an exception.

    :param points: An array of waypoints which are themselves array of the form [x,y,z,r,p,y]
    :param arm: The selected arm (LEFT or RIGHT)
    :param planning_tries: The number of tries allowed for planning (default=100)
    :type points: float[[]]
    :type arm: int
    :type planning_tries: int
    :returns: Nothing
    :rtype: None
    """
    global group_l
    global group_r

    if arm != BOTH:
        plan = plan_path(points, arm, planning_tries)
        if plan is not None:
            if arm == RIGHT:
                group_r.execute(plan)
            else:
                group_l.execute(plan)
    else:
        pass
        # traverse_pathDual(points, planning_tries)


# Plan a trajectory and execute it
def plan_and_move(move_group, target):
    """Plans and moves a group to target

    Creates a plan to move a move_group to the given target. Used by go_to_simple

    :param move_group: The group to move
    :param target: The pose to move to
    :type move_group: MoveGroup
    :type target: PoseStamped
    :returns: Nothing
    :rtype: None
    """
    euler = tf.transformations.euler_from_quaternion(
        (target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w))

    if move_group == group_l:
        arm = 'left'
    if move_group == group_r:
        arm = 'right'

    rospy.loginfo('Planning and moving ' + arm + ' arm: Position: {'
                  + str(target.position.x) + ';' + str(target.position.y) + ';'
                  + str(target.position.z) + '}. Rotation: {' + str(euler[0]) + ';'
                  + str(euler[1]) + ';' + str(euler[2]) + '}.')
    move_group.set_pose_target(target)
    plan = move_group.plan()
    # move_group.go(wait=True)
    # changed
    move_group.execute(plan, wait=True)
    move_group.stop()
    rospy.sleep(3.0)


# Resets a single arm to the reset pose
def reset_arm(arm):
    """Resets an arm

    Resets a single arm to its reset position

    :param arm: The selected arm (LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    safeJointPositionR = [1.6379728317260742, 0.20191457867622375, -2.5927578258514404,
                          0.538416862487793, 2.7445449829101562, 1.5043296813964844, 1.7523150444030762]
    safeJointPositionL = [-1.46564781665802, 0.3302380442619324, 2.507143497467041,
                          0.7764986753463745, -2.852548837661743, 1.659092664718628, 1.378138542175293]
    global group_l
    global group_r
    global group_both

    if arm == RIGHT:
        group_r.set_joint_value_target(safeJointPositionR)
        group_r.plan()
        group_r.go(wait=True)
        gripper_effort(RIGHT, 15.0)
        gripper_effort(RIGHT, 0.0)
    elif arm == LEFT:
        group_l.set_joint_value_target(safeJointPositionL)
        group_l.plan()
        group_l.go(wait=True)
        gripper_effort(LEFT, 15.0)
        gripper_effort(LEFT, 0.0)
    elif arm == BOTH:
        group_both.set_joint_value_target(safeJointPositionL + safeJointPositionR)
        group_both.go(wait=True)
        gripper_effort(LEFT, 15.0)
        gripper_effort(LEFT, 0.0)
        gripper_effort(RIGHT, 15.0)
        gripper_effort(RIGHT, 0.0)

    rospy.sleep(2.0)


def move_both(targetL, targetR):
    """
    Plan and Motion of both arm in the same time given the poses
    :param targetL: Vector for the left arm: x,y,z position and the r,p,y Euler angles
    :param targetR: Vector for the right arm: x,y,z position and the r,p,y Euler angles
    :return: Nothing
    """
    # Conversion of the targets into a pose
    poseL = create_pose_euler(targetL[0], targetL[1], targetL[2], targetL[3], targetL[4], targetL[5])
    poseR = create_pose_euler(targetR[0], targetR[1], targetR[2], targetR[3], targetR[4], targetR[5])
    # Plan for the single arm
    planL = group_l.plan(poseL)
    planR = group_r.plan(poseR)
    # Conversion of the plan messages into a dictionary
    plan_L = message_converter.convert_ros_message_to_dictionary(planL)
    plan_R = message_converter.convert_ros_message_to_dictionary(planR)
    # Take all the points of the joints L and R
    points_L = plan_L['joint_trajectory']['points']
    points_R = plan_R['joint_trajectory']['points']
    # Extract the lists of the lists of joints
    joints_L = [i['positions'] for i in points_L if 'positions' in i]
    joints_R = [i['positions'] for i in points_R if 'positions' in i]
    # Check that the "discretization" of the joint positions is the same
    rospy.loginfo('The number of points for the left arm: {}'.format(len(joints_L)))
    rospy.loginfo('The number of points for the right arm: {}'.format(len(joints_R)))
    if len(joints_L) is len(joints_R):
        for i, j in joints_L, joints_R:
            # rospy.loginfo(i)
            # rospy.loginfo(j)
            group_both.set_goal_joint_tolerance(0.01)
            group_both.set_joint_value_target(i + j)
            group_both.go(wait=True)
    elif len(joints_L) != len(joints_R):
        # Used zip for iterate on the shortest list
        for i, j in zip(joints_L, joints_R):
            # rospy.loginfo(i)
            # rospy.loginfo(j)
            group_both.set_goal_joint_tolerance(0.01)
            group_both.set_joint_value_target(i + j)
            group_both.go(wait=True)
        # TODO: This will remove the synchronism :( => find a new solution
        if len(joints_L) < len(joints_R):
            for i in joints_R[len(joints_L):]:
                group_r.set_goal_joint_tolerance(0.01)
                group_r.set_joint_value_target(i)
                group_r.go(wait=True)
        else:
            for i in joints_L[len(joints_R):]:
                group_l.set_goal_joint_tolerance(0.01)
                group_l.set_joint_value_target(i)
                group_l.go(wait=True)


# Resets the YuMi to a predetermined position
def reset_pose():
    """Resets YuMi

    Moves YuMi arms to an initial joint position with grippers closed

    :returns: Nothing
    :rtype: None
    """

    print("Resetting YuMi arms to an initial joint position with grippers closed")

    reset_arm(BOTH)
