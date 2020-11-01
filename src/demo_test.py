#!/usr/bin/env python

import rospy
import sys
import copy
from moveit_msgs.msg import *
from moveit_commander import *
import std_msgs.msg
import geometry_msgs.msg
from yumi_utils import PI
from yumi_hw.srv import *
from yumi_utils import gripper_effort
from tf.transformations import quaternion_from_euler


# Arm IDs
RIGHT = 1  # :ID of the right arm
LEFT = 2  # :ID of the left arm
BOTH = 3  # :ID of both_arms

# Defining the measure of the table
table_height = 0.025  # [m] :The height of the upper surface of the table (z)
table_length = 1.200  # [m] :The width of the table (y)
table_width = 0.400  # [m] :The width of the table (x)
# Length of the Gripper from datasheet
z_gripper = 0.136  # [m]

# Choice of the planners
# planner = "RRTstarkConfigDefault"  # Asymptotic optimal tree-based planner
planner = "ESTkConfigDefault"  # Default: tree-based planner
# planner = "RRTConnectConfigDefault"  # Tree-based planner
# planner = "PRMstarkConfigDefault"  # Probabilistic Roadmap planner

planning_attempts = 100  # planning attempts
planning_time = 50  # [s] Planning time for computation

# Defining the workspace [min X, min Y, min Z, max X, max Y, max Z]
ws_R = [0.000, -table_length/2, table_height, 0.600, 0.200, 0.593]
ws_L = [0.000, -0.200, table_height, 0.600, table_length, 0.593]


def run():
    # Initialization of Moveit
    rospy.loginfo('Starting the Initialization')
    roscpp_initialize(sys.argv)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    mpr = MotionPlanRequest()
    rospy.sleep(1.0)

    # Left arm
    group_l = MoveGroupCommander("left_arm")
    # Type of planner
    group_l.set_planner_id(planner)
    group_l.set_pose_reference_frame("yumi_body")

    # Setting the workspace
    group_l.set_workspace(ws=ws_L)

    # Replanning
    group_l.allow_replanning(True)
    group_l.set_goal_tolerance(0.005)
    group_l.set_num_planning_attempts(planning_attempts)
    group_l.set_planning_time(planning_time)

    # Right arm
    group_r = MoveGroupCommander("right_arm")
    # Type of planner
    group_r.set_planner_id(planner)
    group_r.set_pose_reference_frame("yumi_body")

    # Setting the workspace
    group_r.set_workspace(ws=ws_R)

    # Replanning
    group_r.allow_replanning(True)
    group_r.set_goal_tolerance(0.005)
    group_r.set_num_planning_attempts(planning_attempts)
    group_r.set_planning_time(planning_time)
    print(group_r.get_joints())
    # Both arms
    group_both = MoveGroupCommander("both_arms")
    # Type of planner
    group_both.set_planner_id(planner)

    # Pose reference frame is the yumi_body
    group_both.set_pose_reference_frame("yumi_body")
    # Replanning
    group_both.allow_replanning(True)
    group_both.set_goal_tolerance(0.005)
    group_both.set_num_planning_attempts(planning_attempts)
    group_both.set_planning_time(planning_time)

    # Publish the trajectory on Rviz
    rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    rospy.sleep(1.0)
    rospy.loginfo('Finished init')

    rospy.loginfo('Open the grippers')
    gripper_effort(LEFT, -20.0)
    # Relaxing gripper
    gripper_effort(LEFT, 0.0)
    gripper_effort(RIGHT, -20.0)
    # Relaxing gripper
    gripper_effort(RIGHT, 0.0)
    # Getting the current state of the robot: RobotState Message
    rospy.loginfo('getting the current state message')
    state = robot.get_current_state()
    group_both.set_start_state(state)

    # Defining the pose target object
    pose_target = geometry_msgs.msg.Pose()
    p_target = [0.31500, -0.20200, 0.172, 0, PI, PI]
    quaternion_target = quaternion_from_euler(p_target[3], p_target[4], p_target[5])
    pose_target.position.x = p_target[0]
    pose_target.position.y = p_target[1]
    pose_target.position.z = p_target[2] + 0.300
    pose_target.orientation.x = quaternion_target[0]
    pose_target.orientation.y = quaternion_target[1]
    pose_target.orientation.z = quaternion_target[2]
    pose_target.orientation.w = quaternion_target[3]
    # Planning to the first position
    rospy.loginfo('Planning and Executing to the first position')
    group_r.set_pose_target(pose_target)
    plan = group_r.plan()
    group_r.execute(plan, wait=True)
    group_r.stop()

    # # Creating a Joint Constraint for the Right Arm
    # joint_constraint = JointConstraint()
    # # Imposing the last joint has 180 degrees of angle
    # joint_constraint.joint_name = group_r.get_joints()[-1]
    # joint_constraint.position = -PI
    # joint_constraint.tolerance_above = 0.005
    # joint_constraint.tolerance_below = 0.005
    # joint_constraint.weight = 1.0
    # joint_constraint_list = [joint_constraint]
    # constraint_list = Constraints()
    # constraint_list.name = 'Picking and Placing joint orientation'
    # constraint_list.joint_constraints = joint_constraint_list
    # group_r.set_path_constraints(constraint_list)

    # Setting the Orientation constraint
    orientation_constraints = OrientationConstraint()
    orientation_constraints.link_name = "gripper_r_base"
    orientation_constraints.header.frame_id = "world"
    orientation_constraints.orientation = copy.deepcopy(pose_target.orientation)
    orientation_constraints.absolute_x_axis_tolerance = 0.1
    orientation_constraints.absolute_y_axis_tolerance = 0.1
    orientation_constraints.absolute_z_axis_tolerance = 0.1
    orientation_constraints.weight = 1.0
    # Constraints should be a list
    orientation_constraint_list = [orientation_constraints]
    # Declaring the object constraints
    constraint_list = Constraints()
    constraint_list.orientation_constraints = orientation_constraint_list
    group_r.set_path_constraints(constraint_list)
    pose_target.position.z = 0.180
    group_r.set_start_state_to_current_state()
    rospy.loginfo('Preparing new motion')
    group_r.set_pose_target(pose_target)
    rospy.sleep(1.0)
    plan2 = group_r.plan()
    print(plan2)
    group_r.execute(plan2, wait=True)
    group_r.clear_path_constraints()


if __name__ == '__main__':
    try:
        # Initialization of the Node
        rospy.init_node('demo_test', anonymous=True)
        run()
        rospy.loginfo('finished')
        roscpp_shutdown()
    except Exception as e:
        print(e)
