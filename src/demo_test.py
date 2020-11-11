#!/usr/bin/env python


import copy
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

# group_r.set_end_effector_link("gripper_r_base")
end_effectorR = group_r.get_end_effector_link()
rospy.loginfo('End effector R: {}'.format(end_effectorR))

group_l.set_end_effector_link("gripper_l_base")
end_effectorL = group_l.get_end_effector_link()
rospy.loginfo('End effector L: {}'.format(end_effectorL))

end_effectors = group_both.get_end_effector_link()
rospy.loginfo('gli end effectors sono: {}'.format(end_effectors))

rospy.loginfo('Finished init')

# clean the scene

rospy.loginfo('Cleaning of the objects in the scene')
rospy.loginfo('Cleaning of the objects')
try:
    scene.remove_world_object("table")
    scene.remove_world_object("input_rack")
except Exception as e:
    print(e)
rospy.sleep(0.5)

# add the table table
rospy.loginfo('Adding the table object')
table_pose = PoseStamped()
table_pose.header.frame_id = "yumi_body"
table_pose.pose.position.x = 0.150 + table_width / 2
table_pose.pose.position.y = 0.0
table_pose.pose.position.z = table_height / 2
scene.add_box("table", table_pose, size=(table_width, 1.2, table_height))

rospy.sleep(0.5)
x_input_rack = 0.260
y_input_rack = 0.175
z_input_rack = 0.075
input_rack_pose = PoseStamped()
input_rack_pose.header.frame_id = "yumi_body"
input_rack_pose.pose.position.x = 0.3465
input_rack_pose.pose.position.y = 0.38090
input_rack_pose.pose.position.z = table_height + 0.075/2
scene.add_box("input_rack", input_rack_pose, size=(0.260, 0.175, 0.075))


home_R = [1.6379728317260742, 0.20191457867622375, -2.5927578258514404, 0.538416862487793, 2.7445449829101562,
          1.5043296813964844, 1.7523150444030762]
home_L = [-1.46564781665802, 0.3302380442619324, 2.507143497467041, 0.7764986753463745, -2.852548837661743,
          1.659092664718628, 1.378138542175293]

home_joints = home_L + home_R

length_tube = 0.125  # [m]

# Points useful: need to compute the pose
# Rendezvous_picking_point: center of the rack at an height of 10 cm more
rendezvous_picking_pose = Pose()
rendezvous_picking_pose.position = copy.deepcopy(input_rack_pose.pose.position)
rendezvous_picking_pose.position.z += 0.100  # [m]

R1 = euler_matrix(0, PI, 0)
R2 = euler_matrix(0, 0, PI / 4)
quaternion_rendezvous_picking = quaternion_from_matrix(concatenate_matrices(R1, R2))
rendezvous_picking_pose.orientation.x = quaternion_rendezvous_picking[0]
rendezvous_picking_pose.orientation.y = quaternion_rendezvous_picking[1]
rendezvous_picking_pose.orientation.z = quaternion_rendezvous_picking[2]
rendezvous_picking_pose.orientation.w = quaternion_rendezvous_picking[3]

# Picking point
pick = Pose()
# TODO: Set the points A1
pick.position = copy.deepcopy(rendezvous_picking_pose.position)
pick.position.x -= 0.115
pick.position.y -= 0.073
pick.orientation = copy.deepcopy(rendezvous_picking_pose.orientation)


def return_home():
    """
    Return to the home position
    :return:
    """
    state = robot.get_current_state()
    group_both.set_start_state(state)
    rospy.loginfo('Return to home for both arms')
    group_both.go(home_joints, wait=True)
    group_both.stop()


def picking():
    state = robot.get_current_state()
    group_both.set_start_state(state)
    group_l.set_pose_target(rendezvous_picking_pose)
    group_l.go(wait=True)

    # Open the gripper
    gripper_effort(LEFT, -10)
    gripper_effort(LEFT, 0)
    # Set the constraints for the picking:
    # Setting the Orientation constraint
    orientation_constraints = OrientationConstraint()
    orientation_constraints.link_name = "gripper_l_base"
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
    group_l.set_path_constraints(constraint_list)

    # Go to pick position
    group_l.set_pose_target(pick)
    group_l.go(wait=True)
    group_l.stop()
    rospy.sleep(1.0)
    pick_up = group_l.get_current_pose()
    pick.position.z = input_rack_pose.pose.position.z + z_input_rack/2 + 0.005
    group_l.set_max_acceleration_scaling_factor(0.10)
    group_l.set_pose_target(pick)
    group_l.go(wait=True)
    group_l.stop()
    # picking
    gripper_effort(RIGHT, 10)
    # go up
    group_l.set_pose_target(pick_up)
    group_l.go(wait=True)
    group_l.clear_path_constraints()
    # returning to rendezvous
    group_l.set_max_velocity_scaling_factor(1.0)
    group_l.set_max_acceleration_scaling_factor(0.50)
    group_l.set_pose_target(rendezvous_picking_pose)
    group_l.go(wait=True)
    group_l.stop()


def run():
    global robot
    global group_l
    global group_r
    global group_both
    global scene
    global mpr

    print('robot get planning frame {}'.format(robot.get_planning_frame()))

    picking()


if __name__ == '__main__':
    try:
        run()
        rospy.loginfo('finished')
        roscpp_shutdown()
    except Exception as e:
        print(e)
