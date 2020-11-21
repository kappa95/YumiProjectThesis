#!/usr/bin/env python


import copy
from typing import Any
from moveit_msgs.msg import *
from moveit_commander import *
from std_msgs.msg import Bool
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

# Replanning
group_l.allow_replanning(True)

# Right arm
group_r = MoveGroupCommander("right_arm")
# Type of planner
group_r.set_planner_id(planner)
group_r.set_pose_reference_frame("yumi_body")

# Setting the workspace
group_r.set_workspace(ws=ws_R)

# Replanning
group_r.allow_replanning(True)

# Both arms
group_both = MoveGroupCommander("both_arms")
# Type of planner
group_both.set_planner_id(planner)

# Pose reference frame is the yumi_body
group_both.set_pose_reference_frame("yumi_body")

# Publish the trajectory on Rviz
rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
rospy.sleep(1.0)


# clean the scene

rospy.loginfo('Cleaning of the objects in the scene')
try:
    scene.remove_world_object("table")
    scene.remove_world_object("output_rack")
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
# Output rack
# Dimensions
x_output_rack = 0.175
y_output_rack = 0.260
z_output_rack = 0.075
# Pose of the center
output_rack_pose = PoseStamped()
output_rack_pose.header.frame_id = "yumi_body"
output_rack_pose.pose.position.x = 0.3465
output_rack_pose.pose.position.y = 0.38090
output_rack_pose.pose.position.z = table_height + z_output_rack/2

# Input rack
# Dimensions
x_input_rack = 0.130  # [m]
y_input_rack = 0.260  # [m]
z_input_rack = 0.060  # [m]
# Pose of the center
input_rack_pose = PoseStamped()
input_rack_pose.header.frame_id = "yumi_body"
input_rack_pose.pose.position.x = 0.4163
input_rack_pose.pose.position.y = 0.0385
input_rack_pose.pose.position.z = table_height + z_output_rack/2

# Points useful: need to compute the pose
# Rendezvous_placing_point: center of the rack at an height of 15 cm more
rendezvous_placing_pose = Pose()
rendezvous_placing_pose.position.x = 0.3465
rendezvous_placing_pose.position.y = 0.38090
rendezvous_placing_pose.position.z = table_height + z_output_rack/2
rendezvous_placing_pose.position.z += 0.150 + z_output_rack + z_gripper  # [m]


# Rotation of 45 degrees of the end effector: method of the matrix rotations
R1 = euler_matrix(0, PI, 0)
R2 = euler_matrix(0, 0, -PI / 4)
quaternion_rendezvous_placing = quaternion_from_matrix(concatenate_matrices(R1, R2))
rendezvous_placing_pose.orientation.x = quaternion_rendezvous_placing[0]
rendezvous_placing_pose.orientation.y = quaternion_rendezvous_placing[1]
rendezvous_placing_pose.orientation.z = quaternion_rendezvous_placing[2]
rendezvous_placing_pose.orientation.w = quaternion_rendezvous_placing[3]

# Rendezvous_placing_point = center of the input rack of an height of 10cm + half of the height
rendezvous_pick_pose = Pose()
rendezvous_pick_pose.position.x = input_rack_pose.pose.position.x
rendezvous_pick_pose.position.y = input_rack_pose.pose.position.y
rendezvous_pick_pose.position.z = input_rack_pose.pose.position.z + z_output_rack + z_gripper + 0.100
# Orientation at 45 deg
rendezvous_pick_pose.orientation = copy.deepcopy(rendezvous_placing_pose.orientation)

# Picking input rack point A1
pick = Pose()
# TODO: Add the 2 dx, dy for the picking pose
pick.position = copy.deepcopy(rendezvous_pick_pose.position)
# pick.position.x -= 0.042
# pick.position.y -= 0.110
# EXPERIMENTAL Values
pick.position.x = 0.36641
pick.position.y = -0.06911
pick.orientation = copy.deepcopy(rendezvous_pick_pose.orientation)

# Same column deltas of the output rack
dx_tube_place = 0.02150  # [m]
dy_tube_place = 0.0130  # [m]

place = Pose()
# TODO: Take the real A1 Place pose
place.position = copy.deepcopy(rendezvous_placing_pose.position)
place.position.x -= 0.0742  # [m]
place.position.y -= 0.1082  # [m]
place.orientation = copy.deepcopy(rendezvous_placing_pose.orientation)

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
    # Divide the steps in 2 parts
    for i in xrange(2):
        wpose.position.x += (dest_pose.position.x - start_pose.position.x) * 0.5
        wpose.position.y += (dest_pose.position.y - start_pose.position.y) * 0.5
        wpose.position.z += (dest_pose.position.z - start_pose.position.z) * 0.5
        waypoints.append(copy.deepcopy(wpose))
        # rospy.logdebug('punto {} e\': \n {}'.format(i, waypoints[i]))
    fraction = 0.0
    attempts = 0
    plan = None
    while fraction < 1.0 and attempts < 8 * planning_attempts:
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


def return_home():
    """
    Return to the home position
    :return:
    """

    # Opening the grippers
    gripper_effort(LEFT, -10)
    gripper_effort(LEFT, 0)

    gripper_effort(RIGHT, -10)
    gripper_effort(RIGHT, 0)

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


def picking_L():
    group_l.set_start_state_to_current_state()
    rospy.logdebug('From homeL to rendezvous picking')
    # Setting the orientation constraint
    oc_L = OrientationConstraint()
    oc_L.link_name = "gripper_l_base"
    oc_L.header.frame_id = "yumi_body"
    oc_L.orientation = copy.deepcopy(rendezvous_pick_pose.orientation)
    oc_L.absolute_x_axis_tolerance = 0.1
    oc_L.absolute_y_axis_tolerance = 0.1
    oc_L.absolute_z_axis_tolerance = 0.1
    oc_L.weight = 1.0
    # Constraints should be a list
    oc_L_list = [oc_L]
    # Declaring the object constraints
    constraint_list_L = Constraints()
    constraint_list_L.orientation_constraints = oc_L_list

    # Going to Rendezvous picking
    group_l.set_pose_target(rendezvous_pick_pose)
    reorient_pick = group_l.plan()
    group_l.execute(reorient_pick, wait=True)
    group_l.stop()
    rospy.logdebug('Going to pick position')
    cartesian(pick, group_l, constraint_list_L)

    # picking
    pick_up = group_l.get_current_pose().pose
    # in picking up should go higher a lot...
    pick_up.position.z += 0.060
    pick_new = copy.deepcopy(pick)
    # risolvo errore logico del pick
    pick_new.position.z = input_rack_pose.pose.position.z + z_input_rack / 2 + 0.020 + z_gripper
    # Compensators
    # pick_compensated.position.y += 0.010

    group_l.set_max_acceleration_scaling_factor(0.10)
    group_l.set_max_velocity_scaling_factor(0.25)
    group_l.set_start_state_to_current_state()

    cartesian(pick_new, group_l, constraint_list_L)

    rospy.logdebug('CHECK MEASUREMENTS')
    # rospy.sleep(20)

    # Closing the fingers
    gripper_effort(LEFT, 5)
    rospy.sleep(0.1)

    print('STATO ATTUALE: \n {}'.format(group_l.get_current_pose().pose.position))
    print('OBIETTIVO: \n {}'.format(pick_new.position))

    # Return to pick up position
    cartesian(pick_up, group_l, constraint_list_L)
    # Return to rendezvous
    rendezvous_up = rendezvous_pick_pose
    rendezvous_up.position.z += 0.050
    cartesian(rendezvous_up, group_l, constraint_list_L)
    group_l.clear_path_constraints()


def rendez_to_scan_L():
    group_l.set_start_state_to_current_state()
    rospy.loginfo('starting from the rendezvous picking position')
    # reorient for barcode Scanning
    rospy.logdebug('reorient for barcode scanning')
    reorient = group_l.get_current_pose()
    reorient.pose.orientation = copy.deepcopy(scan_L.orientation)
    reorient_plan = group_l.plan(reorient)
    group_l.execute(reorient_plan, wait=True)
    # Keep the orientation constraint
    oc_home_L = OrientationConstraint()
    oc_home_L.link_name = "gripper_l_base"
    oc_home_L.header.frame_id = "yumi_body"
    oc_home_L.orientation = copy.deepcopy(group_l.get_current_pose(oc_home_L.link_name).pose.orientation)
    oc_home_L.absolute_x_axis_tolerance = 0.05
    oc_home_L.absolute_y_axis_tolerance = 0.05
    oc_home_L.absolute_z_axis_tolerance = 0.05
    oc_home_L.weight = 1.0
    # Constraints should be a list
    oc_L_list = [oc_home_L]
    # Declaring the object constraints
    constraint_list_L = Constraints()
    constraint_list_L.orientation_constraints = oc_L_list
    group_l.set_path_constraints(constraint_list_L)
    
    # Go to Scan
    group_l.set_pose_target(scan_L)
    plan_rendezvous_scanL = group_l.plan()
    group_l.execute(plan_rendezvous_scanL)
    group_l.stop()
    # cartesian(scan_L, group_l, constraint_list_L)
    group_l.clear_path_constraints()
    # Reorienting before going to rendezvous placing pose
    reorient = group_l.get_current_pose()
    reorient.pose.orientation = copy.deepcopy(rendezvous_placing_pose.orientation)
    reorient = group_l.plan()
    group_l.execute(reorient, wait=True)
    group_l.stop()


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


class BarcodeScanning:
    """BarcodeScanning - Check"""

    def __init__(self, ):
        """Constructor for BarcodeScanning"""
        self.ok_received = False
        self.pub = rospy.Publisher('yumi/barcode/ok', Bool, queue_size=1)
        rospy.Subscriber('barcode/response', Bool, self.callback)

    def callback(self, msg):
        try:
            self.ok_received = msg.data
            rospy.loginfo('demo_test has received: {}'.format(self.ok_received))
        except rospy.ROSException as err:
            rospy.logerr(err)

    def answer_true(self):
        rospy.loginfo('Ok from yumi')
        self.pub.publish(True)

    def answer_false(self):
        rospy.loginfo('No from yumi')
        self.pub.publish(False)


def scanning():
    rospy.loginfo('Starting Scanning')
    # Initialize the BarcodeScanning object which receives and publishes oks from BarcodeManager
    checker = BarcodeScanning()
    # Initialize the boolean value for the ok
    ok_received = checker.ok_received
    # Pre-check before starting the motion
    if ok_received:
        # Answer that the barcode is read
        checker.answer_true()
    else:
        # Initialize an initial joint condition -> change the 6 axis into 0
        group_l.set_start_state_to_current_state()
        init_joints = group_l.get_current_joint_values()
        init_joints[-1] = 0
        group_l.go(init_joints, wait=True)
        group_l.stop()
        while not ok_received:
            rospy.logdebug('entering in while, ok_received is: {}'.format(ok_received))
            # Answer False when is not received the True from the BarcodeManager
            checker.answer_false()
            ok_received = checker.ok_received
            init_joints = group_l.get_current_joint_values()
            # In order to don't be close to the axis limit (-229 deg to +229 deg)
            if init_joints[-1] < PI:
                # Move of 30 deg
                init_joints[-1] += PI/6  # I want to test if 30deg are enough -> reduce time
                group_l.go(init_joints, wait=True)
                group_l.stop()
            else:
                init_joints[-1] = -PI
                group_l.go(init_joints, wait=True)
                group_l.stop()
        checker.answer_true()


def placing_L():
    rospy.loginfo('going to rendezvous placing pose: \n {}'.format(rendezvous_placing_pose))
    # Set the constraints for the placing:
    # Setting the Orientation constraint
    rospy.logdebug('Setting the orientation constraint')
    oc_L = OrientationConstraint()
    oc_L.link_name = "gripper_l_base"
    oc_L.header.frame_id = "yumi_body"
    oc_L.orientation = copy.deepcopy(rendezvous_placing_pose.orientation)
    oc_L.absolute_x_axis_tolerance = 0.1
    oc_L.absolute_y_axis_tolerance = 0.1
    oc_L.absolute_z_axis_tolerance = 0.1
    oc_L.weight = 1.0
    # Constraints should be a list
    oc_L_list = [oc_L]
    # Declaring the object constraints
    constraint_list_L = Constraints()
    constraint_list_L.orientation_constraints = oc_L_list

    rospy.logdebug('going to: rendezvous_placing_pose')
    group_l.set_pose_target(rendezvous_placing_pose)
    reorient = group_l.plan()
    group_l.execute(reorient, wait=True)
    group_l.stop()

    rospy.logdebug('Setted the orientation constraint')
    # Go to place position
    rospy.logdebug('Go to place position: \n {}'.format(place))
    cartesian(place, group_l, constraint_list_L)

    place_up = group_l.get_current_pose().pose
    place.position.z = output_rack_pose.pose.position.z + z_output_rack/2 + 0.050 + z_gripper
    group_l.set_max_velocity_scaling_factor(0.25)
    rospy.logdebug('Going to place bottom')
    cartesian(place, group_l, constraint_list_L)

    # placing: opening gripper
    gripper_effort(LEFT, -20)
    gripper_effort(LEFT, 0)
    rospy.sleep(0.1)
    # go up
    rospy.logdebug('going up')
    cartesian(place_up, group_l, constraint_list_L)
    group_l.clear_path_constraints()

    # returning to rendezvous
    rospy.logdebug('going to rendezvous')
    cartesian(rendezvous_placing_pose, group_l)


def run():
    global robot
    global group_l
    global group_r
    global group_both
    global scene
    global mpr

    group_l.clear_path_constraints()
    group_r.clear_path_constraints()
    group_l.set_max_velocity_scaling_factor(1.0)
    group_r.set_max_velocity_scaling_factor(1.0)
    group_l.set_max_acceleration_scaling_factor(1.0)
    group_r.set_max_acceleration_scaling_factor(1.0)

    return_home()
    picking_L()
    rendez_to_scan_L()
    home_to_scan_R()
    rospy.loginfo('############SCAN#########')
    # Remember to comment when simulate
    # scanning()
    rospy.loginfo('########POSTSCAN#######')
    placing_L()
    return_home()


if __name__ == '__main__':
    try:
        rospy.loginfo('Adding the table and racks objects')
        scene.add_box("table", table_pose, size=(table_width, table_length, table_height))
        scene.add_box("output_rack", output_rack_pose, size=(x_output_rack, y_output_rack, z_output_rack))
        scene.add_box("input_rack", input_rack_pose, size=(x_input_rack, y_input_rack, z_input_rack))
        rospy.sleep(1.0)
        for x in xrange(2):
            run()
        rospy.loginfo('finished')
        roscpp_shutdown()
    except KeyboardInterrupt:
        roscpp_shutdown()
