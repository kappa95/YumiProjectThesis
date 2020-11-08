#!/usr/bin/env python


import copy
from moveit_msgs.msg import *
from moveit_commander import *
from std_msgs.msg import String
import geometry_msgs.msg
from yumi_utils import PI
from yumi_hw.srv import *
from yumi_utils import gripper_effort
from tf.transformations import quaternion_from_euler, concatenate_matrices, euler_matrix, quaternion_from_matrix


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

# Initialization of the Node
rospy.init_node('demo_test', anonymous=True)
# Initialization of Moveit
rospy.loginfo('Starting the Initialization')
roscpp_initialize(sys.argv)

robot_yumi = RobotCommander()
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
group_r.set_goal_tolerance(0.0005)
group_r.set_num_planning_attempts(planning_attempts)
group_r.set_planning_time(planning_time)

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

# clean the scene
rospy.loginfo('Cleaning of the objects in the scene')
rospy.loginfo('Cleaning of the table')
scene.remove_world_object("table")
rospy.sleep(0.5)

# add the table table
rospy.loginfo('Adding the table object')
table_pose = PoseStamped()
table_pose.header.frame_id = robot_yumi.get_planning_frame()
table_pose.pose.position.x = 0.150 + table_width / 2
table_pose.pose.position.y = 0.0
table_pose.pose.position.z = table_height / 2
scene.add_box("table", table_pose, size=(table_width, 1.2, table_height))

home_joints = [-1.407075546891484, -2.0968645586174586, 0.7081984499895384, 0.29709923791025084,
               -5.1407035742911944e-05, -7.242136084961714e-05, -5.345358943031897e-05,
               1.40707559518704, -2.096858749878596, -0.7081717891089125, 0.29710402150605564,
               -9.50240857688911e-05, -5.2985829942286955e-05, -3.199690151589607e-05]

length_tube = 0.125  # [m]

# Points useful: need to compute the pose
# Rendezvous_picking_point
rendezvous_picking_point = [0.31500, -0.20200, 0.172 + length_tube, 0, PI, PI]
rendezvous_picking_pose = Pose()
rendezvous_picking_pose.position.x = rendezvous_picking_point[0]
rendezvous_picking_pose.position.y = rendezvous_picking_point[1]
rendezvous_picking_pose.position.z = rendezvous_picking_point[2]

R1 = euler_matrix(rendezvous_picking_point[3], rendezvous_picking_point[4], rendezvous_picking_point[5])
R2 = euler_matrix(0, 0, PI / 4)
quaternion_rendezvous_picking = quaternion_from_matrix(concatenate_matrices(R1, R2))
rendezvous_picking_pose.orientation.x = quaternion_rendezvous_picking[0]
rendezvous_picking_pose.orientation.y = quaternion_rendezvous_picking[1]
rendezvous_picking_pose.orientation.z = quaternion_rendezvous_picking[2]
rendezvous_picking_pose.orientation.w = quaternion_rendezvous_picking[3]

# Picking point
pick = Pose()
# TODO: Set the points
pick.position = copy.deepcopy(rendezvous_picking_pose.position)
pick.position.z -= 0.010
pick.orientation = copy.deepcopy(rendezvous_picking_pose.orientation)


def return_home():
    """
    Return to the home position
    :return:
    """
    state = robot_yumi.get_current_state()
    group_both.set_start_state(state)
    rospy.loginfo('Return to home for both arms')
    group_both.go(home_joints, wait=True)


def picking():
    state = robot_yumi.get_current_state()
    group_both.set_start_state(state)
    group_r.set_pose_target(rendezvous_picking_pose)
    group_r.go(wait=True)
    # Set the constraints for the picking:
    # Setting the Orientation constraint
    orientation_constraints = OrientationConstraint()
    orientation_constraints.link_name = "gripper_r_base"
    orientation_constraints.header.frame_id = "yumi_body"
    orientation_constraints.orientation = copy.deepcopy(rendezvous_picking_pose.orientation)
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
    group_r.set_pose_target(pick)
    group_r.go(wait=True)
    rospy.sleep(1.0)


def run():
    global robot_yumi
    global group_l
    global group_r
    global group_both
    global scene
    global mpr

    return_home()
    picking()
    # rospy.loginfo('Open the grippers')
    # gripper_effort(LEFT, -20.0)
    # # Relaxing gripper
    # gripper_effort(LEFT, 0.0)
    # gripper_effort(RIGHT, -20.0)
    # # Relaxing gripper
    # gripper_effort(RIGHT, 0.0)
    #
    # # Getting the current state of the robot: RobotState Message
    # rospy.loginfo('getting the current state message')
    # state = robot_yumi.get_current_state()
    # group_both.set_start_state(state)
    #
    # # Defining the pose target object
    # pose_target = Pose()
    # p_target = [0.31500, -0.20200, 0.172, 0, PI, PI]
    # quaternion_target = quaternion_from_euler(p_target[3], p_target[4], p_target[5])
    # pose_target.position.x = p_target[0]
    # pose_target.position.y = p_target[1]
    # pose_target.position.z = p_target[2] + 0.300
    # pose_target.orientation.x = quaternion_target[0]
    # pose_target.orientation.y = quaternion_target[1]
    # pose_target.orientation.z = quaternion_target[2]
    # pose_target.orientation.w = quaternion_target[3]
    # # Planning to the first position
    # rospy.loginfo('Planning and Executing to the first position')
    # group_r.set_pose_target(pose_target)
    # plan = group_r.plan()
    # group_r.execute(plan, wait=True)
    # group_r.stop()

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

    # # Setting the Orientation constraint
    # orientation_constraints = OrientationConstraint()
    # orientation_constraints.link_name = "gripper_r_base"
    # orientation_constraints.header.frame_id = "yumi_body"
    # # orientation_constraints.header.frame_id = "world"
    # orientation_constraints.orientation = copy.deepcopy(pose_target.orientation)
    # orientation_constraints.absolute_x_axis_tolerance = 0.1
    # orientation_constraints.absolute_y_axis_tolerance = 0.1
    # orientation_constraints.absolute_z_axis_tolerance = 0.1
    # orientation_constraints.weight = 1.0
    # # Constraints should be a list
    # orientation_constraint_list = [orientation_constraints]
    # # Declaring the object constraints
    # constraint_list = Constraints()
    # constraint_list.orientation_constraints = orientation_constraint_list
    # group_r.set_path_constraints(constraint_list)
    # pose_target.position.z = 0.180
    # group_r.set_start_state_to_current_state()
    # rospy.loginfo('Preparing new motion')
    # group_r.set_pose_target(pose_target)
    # rospy.sleep(1.0)
    # plan2 = group_r.plan()
    # print(plan2)
    # group_r.execute(plan2, wait=True)
    # group_r.clear_path_constraints()


if __name__ == '__main__':
    try:
        run()
        rospy.loginfo('finished')
        roscpp_shutdown()
    except Exception as e:
        print(e)
