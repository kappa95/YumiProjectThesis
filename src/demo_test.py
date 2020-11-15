#!/usr/bin/env python


import copy
from typing import Any
from moveit_msgs.msg import *
from moveit_commander import *
from std_msgs.msg import String
import geometry_msgs.msg
from yumi_utils import PI, gripper_effort
from yumi_hw.srv import *
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

# Distance of the center of the cam from yumi_link_7_r
z_cam = 0.040  # [m]

# Length of the test tube
length_tube = 0.125  # [m]

# Choice of the planners
# planner = "RRTstarkConfigDefault"  # Asymptotic optimal tree-based planner
# planner = "ESTkConfigDefault"  # Default: tree-based planner
planner = "RRTConnectConfigDefault"  # Tree-based planner
# planner = "PRMstarkConfigDefault"  # Probabilistic Roadmap planner

planning_attempts = 100  # planning attempts
planning_time = 20  # [s] Planning time for computation

# Defining the workspace [min X, min Y, min Z, max X, max Y, max Z]
ws_R = [0.000, -table_length/2, table_height, 0.600, 0.200, 0.593]
ws_L = [0.000, -0.200, table_height, 0.600, table_length, 0.593]

# Initialization of the Node
rospy.init_node('demo_test', anonymous=True, log_level=rospy.DEBUG)
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
# group_l.set_num_planning_attempts(planning_attempts)
# group_l.set_planning_time(planning_time)

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


# clean the scene

rospy.loginfo('Cleaning of the objects in the scene')
try:
    scene.remove_world_object("table")
    scene.remove_world_object("input_rack")
except Exception as e:
    print(e)

rospy.sleep(0.5)

# table informations
table_pose = PoseStamped()
table_pose.header.frame_id = "yumi_body"
table_pose.pose.position.x = 0.150 + table_width / 2
table_pose.pose.position.y = 0.0
table_pose.pose.position.z = table_height / 2


# Rack informations
x_input_rack = 0.175
y_input_rack = 0.260
z_input_rack = 0.075
input_rack_pose = PoseStamped()
input_rack_pose.header.frame_id = "yumi_body"
input_rack_pose.pose.position.x = 0.3465
input_rack_pose.pose.position.y = 0.38090
input_rack_pose.pose.position.z = table_height + z_input_rack/2


# Points useful: need to compute the pose
# Rendezvous_picking_point: center of the rack at an height of 10 cm more
rendezvous_picking_pose = Pose()
rendezvous_picking_pose.position.x = 0.3465
rendezvous_picking_pose.position.y = 0.38090
rendezvous_picking_pose.position.z = table_height + z_input_rack/2
rendezvous_picking_pose.position.z += 0.100 + z_input_rack + z_gripper  # [m]

# Rotation of 45 degrees of the end effector: method of the matrix rotations
R1 = euler_matrix(0, PI, 0)
R2 = euler_matrix(0, 0, -PI / 4)
quaternion_rendezvous_picking = quaternion_from_matrix(concatenate_matrices(R1, R2))
rendezvous_picking_pose.orientation.x = quaternion_rendezvous_picking[0]
rendezvous_picking_pose.orientation.y = quaternion_rendezvous_picking[1]
rendezvous_picking_pose.orientation.z = quaternion_rendezvous_picking[2]
rendezvous_picking_pose.orientation.w = quaternion_rendezvous_picking[3]

# Picking input rack point A1
# TODO: Think how to repeat programmatically the placing process
# Same column deltas of the output rack
dx_tube = 0.02150  # [m]
dy_tube = 0.0130  # [m]
pick = Pose()
# TODO: TEST!!!! A1 POSE
# TODO: Convert these positions for the output rack instead of the input
pick.position = copy.deepcopy(rendezvous_picking_pose.position)
pick.position.x -= 0.0742  # [m]
pick.position.y -= 0.1082  # [m]
# pick.position.x -= 0.115
# pick.position.y -= 0.073
pick.orientation = copy.deepcopy(rendezvous_picking_pose.orientation)

# Home points
home_L = [0.300, 0.250, 0.380, 0, PI, 0]
home_R = [0.300, -0.250, 0.380, 0, PI, PI]
q_home_L = quaternion_from_euler(home_L[3], home_L[4], home_L[5])
q_home_R = quaternion_from_euler(home_R[3], home_R[4], home_R[5])

# Distance between the label of the test tube and the camera on z axis
dz_scan = length_tube/2 + z_cam

# Defining the Scan Pose starting from home
scan_L = Pose()
scan_L.position.x = home_L[0]
scan_L.position.y = home_L[1] - 0.125
scan_L.position.z = home_L[2] + dz_scan
scan_L.orientation.x = q_home_L[0]
scan_L.orientation.y = q_home_L[1]
scan_L.orientation.z = q_home_L[2]
scan_L.orientation.w = q_home_L[3]

scan_R = Pose()
scan_R.position.x = home_R[0]
scan_R.position.y = home_R[1] + 0.125
scan_R.position.z = home_R[2]
scan_R.orientation.x = q_home_R[0]
scan_R.orientation.y = q_home_R[1]
scan_R.orientation.z = q_home_R[2]
scan_R.orientation.w = q_home_R[3]

rospy.loginfo('Finished init')


def return_home():
    """
    Return to the home position
    :return:
    """
    group_l.set_start_state_to_current_state()
    group_l.set_pose_target(home_L)
    group_l.go(wait=True)
    group_l.stop()
    group_l.clear_pose_target(group_l.get_end_effector_link())
    group_r.set_start_state_to_current_state()
    group_r.set_pose_target(home_R)
    group_r.go(wait=True)
    group_r.stop()
    group_r.clear_pose_target(group_r.get_end_effector_link())


def cartesian(dest_pose, group, constraint=None):
    # type: (geometry_msgs.msg.Pose , MoveGroupCommander, Any) -> None
    """
    Create a cartesian path with a middle point
    :param dest_pose: Pose
    :param group: MoveGroupCommander
    :param constraint: Path Constraint
    :return: Nothing
    """
    waypoints = []
    # Initialize the waypoints: Pose object
    wpose = copy.deepcopy(group.get_current_pose().pose)
    # Initialize start pose: Pose object
    start_pose = copy.deepcopy(group.get_current_pose().pose)
    # Divide the steps in 4 parts
    for i in xrange(4):
        wpose.position.x += (dest_pose.position.x - start_pose.position.x) * 0.25
        wpose.position.y += (dest_pose.position.y - start_pose.position.y) * 0.25
        wpose.position.z += (dest_pose.position.z - start_pose.position.z) * 0.25
        waypoints.append(copy.deepcopy(wpose))
        rospy.logdebug('punto {} e\': \n {}'.format(i, waypoints[i]))
    fraction = 0.0
    attempts = 0
    plan = None
    while fraction < 1.0 and attempts < 5 * planning_attempts:
        attempts += 1
        (plan, fraction) = group.compute_cartesian_path(waypoints,
                                                        0.01,  # eef step: 1cm
                                                        jump_threshold=0.0,
                                                        avoid_collisions=True,
                                                        path_constraints=constraint)
        # rospy.logdebug('attempts: {} fraction: {}%'.format(attempts, fraction*100))
    # ricalcolare il time della traiettoria!
    if fraction == 1.0:
        plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
        rospy.loginfo('executing the plan')
        rospy.logdebug('attempts: {} fraction: {}%'.format(attempts, fraction * 100))
        group.execute(plan, wait=True)
        group.stop()
    else:
        rospy.logerr('it doesn\'t complete the trajectory, fraction: {}%'.format(fraction*100))
        raise Exception('Exceeded the maximum number of retries')


def picking_L():
    rospy.loginfo('going to rendezvous picking pose: \n {}'.format(rendezvous_picking_pose))
    group_l.set_start_state_to_current_state()

    # Open the gripper
    gripper_effort(LEFT, -10)
    gripper_effort(LEFT, 0)

    # Set the constraints for the picking:
    # Setting the Orientation constraint
    rospy.logdebug('Setting the orientation constraint')
    oc_L = OrientationConstraint()
    oc_L.link_name = "gripper_l_base"
    oc_L.header.frame_id = "yumi_body"
    oc_L.orientation = copy.deepcopy(rendezvous_picking_pose.orientation)
    oc_L.absolute_x_axis_tolerance = 0.1
    oc_L.absolute_y_axis_tolerance = 0.1
    oc_L.absolute_z_axis_tolerance = 0.1
    oc_L.weight = 1.0
    # Constraints should be a list
    oc_L_list = [oc_L]
    # Declaring the object constraints
    constraint_list_L = Constraints()
    constraint_list_L.orientation_constraints = oc_L_list

    group_l.set_max_velocity_scaling_factor(1.0)
    group_l.set_max_acceleration_scaling_factor(1.0)
    group_l.set_pose_target(rendezvous_picking_pose)
    reorient = group_l.plan()
    group_l.execute(reorient, wait=True)
    rospy.logdebug('Setted the orientation constraint')
    # Go to pick position
    rospy.logdebug('Go to pick position: \n {}'.format(pick))
    cartesian(pick, group_l, constraint_list_L)

    # FIXME: This motion is problematic
    pick_up = group_l.get_current_pose().pose
    pick.position.z = input_rack_pose.pose.position.z + z_input_rack/2 + 0.010 + z_gripper
    group_l.set_max_velocity_scaling_factor(0.25)
    group_l.set_start_state_to_current_state()
    cartesian(pick, group_l, constraint_list_L)

    # picking: closing gripper
    gripper_effort(LEFT, 10)

    # go up
    rospy.loginfo('going up')
    cartesian(pick_up, group_l, constraint_list_L)
    group_l.clear_path_constraints()

    # returning to rendezvous
    rospy.loginfo('going to rendezvous')
    group_l.set_max_velocity_scaling_factor(1.0)
    group_l.set_max_acceleration_scaling_factor(0.50)
    cartesian(rendezvous_picking_pose, group_l)


def rendez_to_scan_L():
    # TODO: Add the constraint of the workspace
    group_l.set_start_state_to_current_state()
    rospy.loginfo('starting from the rendezvous picking position')
    # reorient for barcode Scanning
    rospy.logdebug('reorient for barcode scanning')

    reorient = group_l.get_current_joint_values()
    reorient[-1] += PI/4
    group_l.set_joint_value_target(reorient)
    # group_l.go(reorient, wait=True)
    # group_l.stop()

    reorient_plan = group_l.plan(reorient)
    group_l.execute(reorient_plan, wait=True)

    # Keep the orientation constraint
    oc_home_L = OrientationConstraint()
    oc_home_L.link_name = "gripper_l_base"
    oc_home_L.header.frame_id = "yumi_body"
    oc_home_L.orientation = copy.deepcopy(group_l.get_current_pose(oc_home_L.link_name).pose.orientation)
    oc_home_L.absolute_x_axis_tolerance = 0.1
    oc_home_L.absolute_y_axis_tolerance = 0.1
    oc_home_L.absolute_z_axis_tolerance = 0.1
    oc_home_L.weight = 1.0
    # Constraints should be a list
    oc_L_list = [oc_home_L]
    # Declaring the object constraints
    constraint_list_L = Constraints()
    constraint_list_L.orientation_constraints = oc_L_list

    # Go to home
    group_l.set_path_constraints(constraint_list_L)
    group_l.set_start_state_to_current_state()
    group_l.set_pose_target(home_L)
    plan_rendezvous_home = group_l.plan()
    group_l.execute(plan_rendezvous_home)
    group_l.stop()

    # Go to Scan
    cartesian(scan_L, group_l, constraint_list_L)


def home_to_scan_R():
    # Set the constraints for the scan:
    # Setting the Orientation constraint
    rospy.logdebug('Setting the orientation constraint')
    oc_R = OrientationConstraint()
    oc_R.link_name = "gripper_r_base"
    oc_R.header.frame_id = "yumi_body"
    oc_R.orientation = copy.deepcopy(scan_R.orientation)
    oc_R.absolute_x_axis_tolerance = 0.1
    oc_R.absolute_y_axis_tolerance = 0.1
    oc_R.absolute_z_axis_tolerance = 0.1
    oc_R.weight = 1.0
    # Constraints should be a list
    oc_R_list = [oc_R]
    # Declaring the object constraints
    constraint_list_R = Constraints()
    constraint_list_R.orientation_constraints = oc_R_list
    group_r.set_start_state_to_current_state()
    cartesian(scan_R, group_r, constraint=constraint_list_R)


def tube_exchange():
    # Set the constraints for the scan:
    # Setting the Orientation constraint
    rospy.logdebug('Setting the orientation constraint')
    oc_R = OrientationConstraint()
    oc_R.link_name = "gripper_r_base"
    oc_R.header.frame_id = "yumi_body"
    oc_R.orientation = copy.deepcopy(scan_R.orientation)
    oc_R.absolute_x_axis_tolerance = 0.1
    oc_R.absolute_y_axis_tolerance = 0.1
    oc_R.absolute_z_axis_tolerance = 0.1
    oc_R.weight = 1.0
    # Constraints should be a list
    oc_R_list = [oc_R]
    # Declaring the object constraints
    constraint_list_R = Constraints()
    constraint_list_R.orientation_constraints = oc_R_list
    group_r.shift_pose_target(1, -0.050)
    # group_r.set_path_constraints(constraint_list_R)
    plan_homeR_pose = group_r.plan()
    group_r.execute(plan_homeR_pose)
    group_r.stop()
    # group_r.clear_path_constraints()


def run():
    global robot
    global group_l
    global group_r
    global group_both
    global scene
    global mpr

    rospy.loginfo('Adding the table and input rack objects')
    scene.add_box("table", table_pose, size=(table_width, 1.2, table_height))
    scene.add_box("input_rack", input_rack_pose, size=(x_input_rack, y_input_rack, z_input_rack))
    rospy.sleep(1.0)

    return_home()
    picking_L()
    rendez_to_scan_L()
    home_to_scan_R()
    tube_exchange()


if __name__ == '__main__':
    run()
    rospy.loginfo('finished')
    roscpp_shutdown()
