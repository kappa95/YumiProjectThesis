#!/usr/bin/env python

"""It works with a different modification in the moveit config of the KTH
New modification: Changed the parameter in the trajectory_execution.launch.xml"""


from copy import deepcopy
from typing import List
from moveit_msgs.msg import *
from moveit_commander import *
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray, Marker
# from rospy_message_converter import message_converter
import sys
import tf
from os import path
from time import strftime, localtime
from geometry_msgs.msg import *
from yumi_utils import PI, gripper_effort, LEFT, RIGHT
from yumi_hw.srv import *
from trac_ik_python.trac_ik import IK
import matplotlib.pyplot as plt
from matplotlib.figure import Figure, Axes, rcParams

# from moveit_msgs.msg import MoveItErrorCodes
#
#
# moveit_error_dict = {}
#
# for n in MoveItErrorCodes.__dict__.keys():
#     if not n[:1] == '_':
#         code = MoveItErrorCodes.__dict__[n]
#         moveit_error_dict[code] = n

# Set Latex font
rcParams['text.usetex'] = True
rcParams['text.latex.unicode'] = True
# Initialization of Moveit
rospy.loginfo('Starting the Initialization')
roscpp_initialize(sys.argv)
# Initialization of the Node
rospy.init_node('test', anonymous=True, log_level=rospy.INFO)

robot = RobotCommander()
scene = PlanningSceneInterface()
mpr = MotionPlanRequest()
rospy.sleep(1.0)

planner = "RRT"
group_both = MoveGroupCommander("fede_both")
group_both.set_pose_reference_frame("yumi_body")
group_both.set_planner_id(planner)
group_both.set_planning_time(20)
group_both.set_num_planning_attempts(100)
group_both.allow_replanning(False)  # Allow the replanning if there are changes in environment

group_right = MoveGroupCommander("right_arm")
group_right.set_pose_reference_frame("yumi_body")
group_right.set_planning_time(20)
group_right.set_num_planning_attempts(10)
group_right.allow_replanning(True)

# Arms end effectors
right_arm = "yumi_link_7_r"
left_arm = "yumi_link_7_l"
group_left_gripper = 'left_gripper'
group_right_gripper = 'right_gripper'

# Using the TracIK wrapper for having IK
ik_solver_right = IK("yumi_body", right_arm)
sol_ik = ik_solver_right.get_ik(
    group_right.get_current_joint_values(),  # Current state as seed
    0.300, -0.300, 0.380,  # Position
    0.0, 0.0, 0.0, 1.0,  # Quaternion orientation
    0.01, 0.01, 0.01,  # Linear tolerance X, Y, Z
    0.1, 0.1, 0.1  # Angular tolerance X, Y, Z
)

print(sol_ik)

# Path for saving the images:
images_path = path.join(path.expanduser("~"), "Thesis/Images/{}/".format(planner))

# Publish the trajectory on Rviz
# rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
# rospy.sleep(1.0)


# Home position
home_L = [0.300, 0.300, 0.380, 0, PI, 0]
home_R = [0.300, -0.300, 0.380, 0, PI, PI]

# Adding the test tube
tube_length = 0.1030
tube_radius = 0.0180
base_height = 1.0
test_tube_pose = PoseStamped()
test_tube_pose.header.frame_id = "base"
test_tube_pose.pose.position.x = 0.0
test_tube_pose.pose.position.y = 0.0
test_tube_pose.pose.position.z = (tube_length + base_height)/2


class TestTube:
    """
    Defines the object test tube
    """
    count = 0  # Counter of the instances for naming the object

    def __init__(self, name=None, x=0.0, y=0.0):
        # type: (str, float, float) -> None
        self.length = 0.1030
        self.radius = 0.0180
        self.height = 1.0
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "base"
        self.pose_msg.pose.position.x = x
        self.pose_msg.pose.position.y = y
        self.pose_msg.pose.position.z = (tube_length + base_height) / 2
        self.id = TestTube.count
        TestTube.count += 1
        if name is not None:
            self.name = "test_tube_" + str(name)
        else:
            self.name = "test_tube_" + str(TestTube.count + 1)

    def add_object(self):
        scene.add_box(self.name, self.pose_msg, size=(self.radius, self.radius, self.length))
        rospy.sleep(0.5)

    def remove_object(self):
        if scene.get_attached_objects(self.name):
            rospy.logerr('Object attached: check it please!')
        else:
            scene.remove_world_object(self.name)

    def attach_object(self, arm):
        # type: (TestTube, str) -> None
        if arm is left_arm:
            touch_links = robot.get_link_names(group_left_gripper)
        else:
            touch_links = robot.get_link_names(group_right_gripper)
        scene.attach_box(arm, self.name, touch_links=touch_links)
        rospy.sleep(1.0)

    def detach_object(self, arm):
        # type: (TestTube, str) -> None
        scene.remove_attached_object(arm, self.name)
        rospy.sleep(0.5)


# Place positions are subscribed by the placepos topic
class PlaceSub:
    def __init__(self):
        self.placeMarkers = []  # type: List[Pose]
        self.msg = rospy.wait_for_message("/placepos", topic_type=MarkerArray)  # type: MarkerArray
        self.placeMarkers = [j for j in self.msg.markers]  # type: List[Marker]


# Take Place positions
placeSub = PlaceSub()
# Place positions
# TODO: Check that probably here there are the positions of the buffers.
placePS = []
for i in placeSub.placeMarkers:
    temp = PoseStamped()
    temp.header.frame_id = i.header.frame_id
    temp.pose = i.pose
    temp.pose.orientation = group_both.get_current_pose(left_arm).pose.orientation
    temp.pose.position.z += tube_length / 2 + 0.136 + 0.01 + 0.03002
    placePS.append(temp)


def getting_joints_from_plan(plan):
    # type: (RobotTrajectory) -> list
    """
    Provide the joints of the last point of the trajectory

    :param plan: RobotTrajectory msg: Plan of a trajectory
    :type plan: RobotTrajectory
    :rtype: list
    :return: List of joints of the end of the trajectory
    """
    positions = plan.joint_trajectory.points[-1]  # type: JointTrajectoryPoint
    # plan_dict = message_converter.convert_ros_message_to_dictionary(plan)  # type: dict
    # positions = plan_dict['joint_trajectory']['points'][-1]['positions']
    return positions.positions


def create_robotstate(plan):
    """
    Return a RobotState() msg of the end of a trajectory

    :param plan: RobotTrajectory msg of the trajectory - planning
    :type plan: RobotTrajectory
    :rtype: RobotState
    :return: robot_state: RobotState msg of the trajectory
    """
    if plan.joint_trajectory.points:
        # Creating a RobotState for evaluate the next trajectory
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = plan.joint_trajectory.joint_names

        positions = getting_joints_from_plan(plan)
        joint_state.position = positions
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        return robot_state
    else:
        raise RuntimeWarning("Error in creating the robotstate: points empty")


def evaluate_time(plan, info=''):
    """
    It returns the time duration of the trajectory

    :param plan: Plan msg of the trajectory
    :type plan: RobotTrajectory
    :param info: info about the time
    :type info: str
    :rtype: float
    :return: duration time of the trajectory
    """
    # Check if the plan is not empty
    if plan.joint_trajectory.points:
        p_last = plan.joint_trajectory.points[-1]  # type: JointTrajectoryPoint
        duration = p_last.time_from_start.to_sec()
        # plan_dict = message_converter.convert_ros_message_to_dictionary(plan)  # type: dict
        # duration_time = plan_dict['joint_trajectory']['points'][-1]['time_from_start']  # type: dict
        # duration = float(duration_time['nsecs'])*10**(-9)  # type: float
        rospy.loginfo('Estimated time of planning {}: {} s'.format(info, duration))
        return duration
    else:
        raise RuntimeWarning("Plan is empty")
        # rospy.logwarn('The plan is empty')
        # duration = 0.0
        # return duration


def home():
    rospy.loginfo('home')
    group_both.set_pose_target(home_L, left_arm)
    group_both.set_pose_target(home_R, right_arm)
    plan = group_both.plan()
    home_duration = evaluate_time(plan, "homing")
    group_both.execute(plan)
    group_both.stop()
    return home_duration


def picking(obj, arm, info=''):
    # type: (TestTube, str, str) -> List[float, RobotTrajectory, RobotTrajectory]
    """
    Wrapper for picking

    :param obj: Object to pick
    :param arm: Arm used
    :param info Info about what is doing
    :rtype: list[float, RobotTrajectory, RobotTrajectory] or tuple[float, RobotTrajectory, RobotTrajectory]
    :return: Duration time for picking and RobotTrajectories for picking
    """
    pose_P = deepcopy(obj.pose_msg)
    pose_P.pose.position.z += tube_length/2 + 0.12
    pose_P.pose.orientation = group_both.get_current_pose(arm).pose.orientation

    if arm is left_arm:
        home_pose = home_L
        grip = LEFT
    else:
        # home_pose = home_R
        home_pose = placePS[3]
        grip = RIGHT

    gripper_effort(grip, -20)
    group_both.set_pose_target(pose_P, arm)
    pick = group_both.plan()
    # Evaluate the time for picking
    t1 = evaluate_time(pick, info + "_t1")
    if pick.joint_trajectory.points:
        # Creating a RobotState for evaluate the next trajectory
        robot_state = create_robotstate(pick)
        group_both.set_start_state(robot_state)
        group_both.set_pose_target(home_pose, arm)
        homing = group_both.plan()
        # Evaluate the duration of the planning
        t2 = evaluate_time(homing, info + "_t2")
        return [(t1 + t2), pick, homing]
    else:
        rospy.logerr('Planning failed')
        pass


def placing(obj, info=''):
    # type: (TestTube, str) -> List[float, RobotTrajectory, RobotTrajectory]
    idx = obj.id  # Index of the test tube from the class
    group_both.set_pose_target(placePS[idx], left_arm)
    placePlan = group_both.plan()  # type: RobotTrajectory
    # Evaluate the time of the trajectory
    t1 = evaluate_time(placePlan, info + "_t1_placing")
    if placePlan.joint_trajectory.points:
        # Creating a RobotState for evaluate the next trajectory
        group_both.clear_pose_targets()
        place_state = create_robotstate(placePlan)
        group_both.set_start_state(place_state)
        group_both.set_pose_target(home_L, left_arm)
        return_home = group_both.plan()  # type: RobotTrajectory
        t2 = evaluate_time(return_home, info + "_t2_placing")  # type: float
        return [(t1 + t2), placePlan, return_home]
    else:
        raise RuntimeWarning("Error: Planning failed for placing")
        # rospy.logerr('Planning failed')
        # pass


def placing_both(obj_place, obj_pick, info=''):
    # type: (TestTube, TestTube, str) -> List[float, RobotTrajectory, RobotTrajectory]
    # Pose of the picking object
    pose_P = deepcopy(obj_pick.pose_msg)
    pose_P.pose.position.z += tube_length/2 + 0.12
    pose_P.pose.orientation = group_both.get_current_pose(right_arm).pose.orientation

    idx = obj_place.id  # Index of the test tube to place
    # Left arm should place the object and in the meanwhile the right arm picking the next
    gripper_effort(RIGHT, -20)
    group_both.set_pose_target(placePS[idx], left_arm)
    group_both.set_pose_target(pose_P, right_arm)
    placePlan = group_both.plan()
    # Evaluate the time of the trajectory
    t1 = evaluate_time(placePlan, info + "t1_placePlan_Placeboth")
    if placePlan.joint_trajectory.points:
        # Creating a RobotState for evaluate the next trajectory
        group_both.clear_pose_targets()
        place_state = create_robotstate(placePlan)
        group_both.set_start_state(place_state)
        group_both.set_pose_target(home_L, left_arm)
        group_both.set_pose_target(placePS[3], right_arm)
        return_home = group_both.plan()
        t2 = evaluate_time(return_home, info + "t2_returnHomePlaceboth")
        return [(t1 + t2), placePlan, return_home]
    else:
        rospy.logerr('Planning failed')
        pass


def joint_diagram(plan, info=''):
    # type: (RobotTrajectory, str) -> None
    points = [p for p in plan.joint_trajectory.points]  # type: List[JointTrajectoryPoint]
    # Create an object figure subplot
    fig, axes = plt.subplots(3, sharex=True)  # type: Figure, List[Axes]
    # Get width and height
    (width_fig, height_fig) = rcParams["figure.figsize"]
    # Get hspace between subplots + 20%
    hspace = rcParams["figure.subplot.hspace"] + 0.20
    # Add half inch to the figure's size
    fig.set_size_inches(width_fig + 1, height_fig + 1.1)
    fig.subplots_adjust(hspace=hspace)
    # For each point, I separate each joint position
    t = [tt.time_from_start.to_sec() for tt in points]
    j1_l = [jj.positions[0] for jj in points]
    j2_l = [jj.positions[1] for jj in points]
    j7_l = [jj.positions[2] for jj in points]
    j3_l = [jj.positions[3] for jj in points]
    j4_l = [jj.positions[4] for jj in points]
    j5_l = [jj.positions[5] for jj in points]
    j6_l = [jj.positions[6] for jj in points]
    axes[0].plot(
        t, j1_l, 'bo-',
        t, j2_l, 'go-',
        t, j7_l, 'ro-',
        t, j3_l, 'co-',
        t, j4_l, 'mo-',
        t, j5_l, 'yo-',
        t, j6_l, 'ko-'
    )
    axes[0].grid()
    # axes[0].set_title(r"\text{Joint positions - left arm - plan: {0}}".format(info))
    axes[0].set_title(r"$\textbf{Joint positions - left arm - plan: %s}$" % info)
    axes[0].set_xlabel(r"$\textit{time (s)}$")
    axes[0].set_ylabel(r"$q$")
    axes[0].legend(['j1', 'j2', 'j7', 'j3', 'j4', 'j5', 'j6'], loc='best', bbox_to_anchor=(1.001, 1))
    v1_l = [jj.velocities[0] for jj in points]
    v2_l = [jj.velocities[1] for jj in points]
    v7_l = [jj.velocities[2] for jj in points]
    v3_l = [jj.velocities[3] for jj in points]
    v4_l = [jj.velocities[4] for jj in points]
    v5_l = [jj.velocities[5] for jj in points]
    v6_l = [jj.velocities[6] for jj in points]
    axes[1].plot(
        t, v1_l, 'bo-',
        t, v2_l, 'go-',
        t, v7_l, 'ro-',
        t, v3_l, 'co-',
        t, v4_l, 'mo-',
        t, v5_l, 'yo-',
        t, v6_l, 'ko-'
    )
    axes[1].grid()
    axes[1].set_xlabel(r"$\textit{time (s)}$")
    axes[1].set_ylabel(r"$\dot{q}$")
    # axes[1].set_title(r"\text{Joint speed - left arm - plan: {0}}".format(info))
    axes[1].set_title(r"$\textbf{Joint speed - left arm - plan: %s}$" % info)
    a1_l = [jj.accelerations[0] for jj in points]
    a2_l = [jj.accelerations[1] for jj in points]
    a7_l = [jj.accelerations[2] for jj in points]
    a3_l = [jj.accelerations[3] for jj in points]
    a4_l = [jj.accelerations[4] for jj in points]
    a5_l = [jj.accelerations[5] for jj in points]
    a6_l = [jj.accelerations[6] for jj in points]
    axes[2].plot(
        t, a1_l, 'bo-',
        t, a2_l, 'go-',
        t, a7_l, 'ro-',
        t, a3_l, 'co-',
        t, a4_l, 'mo-',
        t, a5_l, 'yo-',
        t, a6_l, 'ko-'
    )
    axes[2].grid()
    axes[2].set_xlabel(r"$\textit{time (s)}$")
    axes[2].set_ylabel(r"$\ddot{q}$")
    # axes[2].set_title(r"\text{Joint acceleration - left arm - plan: {0}}".format(info))
    axes[2].set_title(r"$\textbf{Joint acceleration - left arm - plan: %s}$" % info)
    # print("end time: {}".format(t[-1]))
    fig.savefig(images_path + "J_left_arm_{}_{}".format(info, strftime("%d_%b-%H_%M", localtime())),
                format='svg',
                transparent=False
                )
    # plt.show()
    # Save the timings on a file:
    with open(path.join(path.expanduser("~"), "Thesis/timings"), "a+") as f:
        f.write(
            strftime("%d_%b-%H_%M", localtime()) +
            "\tPLANNER: {} J_left_arm_{} Time: {}\n".format(planner, info, t[-1])
        )


def run():
    # Open grippers
    gripper_effort(LEFT, -20)
    gripper_effort(RIGHT, -20)
    # Remove detached object
    if scene.get_attached_objects():
        scene.remove_attached_object(left_arm)
        scene.remove_attached_object(right_arm)
    # Remove all the objects
    scene.remove_world_object("test_tubes")
    rospy.sleep(1.0)

    # Add the test tube
    T1 = TestTube()
    rospy.loginfo('ID del tube1: {}'.format(T1.id))
    T2 = TestTube(y=0.03)
    rospy.loginfo('ID del tube2: {}'.format(T2.id))

    rospy.loginfo('Placing tubes')
    # Setting up the test tubes
    T1.add_object()
    T2.add_object()
    rospy.sleep(2.0)

    # Going home
    group_both.set_start_state_to_current_state()
    home()

    # # Evaluate the time for the LEFT arm cycle
    (t1_L, pick_L, homing_L) = picking(T1, left_arm, info="pick1_L\t")
    home_robotstate = create_robotstate(homing_L)
    group_both.set_start_state(home_robotstate)
    (t2_L, place_L, return_home_L) = placing(T1, info="place1_L\t")
    # Create the initial state after placing the tube
    return_home_L_state = create_robotstate(return_home_L)
    group_both.set_start_state(return_home_L_state)
    # 2nd test tube
    (t3_L, pick2_L, homing_L2) = picking(T2, left_arm, info="pick2_L\t")
    home2_robotstate = create_robotstate(homing_L2)
    group_both.set_start_state(home2_robotstate)
    (t4_L, place2_L, return_home2_L) = placing(T2, "place2_L\t")
    duration_L = t1_L + t2_L + t3_L + t4_L

    # # Evaluate the time for the RIGHT arm cycle: pick + buffer + pick2 + place
    (t1_R, pick_R, buffer_R) = picking(T1, right_arm, info="pick1_R\t")
    buffer_R_state = create_robotstate(buffer_R)
    # Picking from buffer for left arm
    group_both.set_start_state(buffer_R_state)
    group_both.clear_pose_targets()
    group_both.set_pose_target(placePS[3], left_arm)
    group_both.set_pose_target(home_R, right_arm)
    # Planning the exchange
    buffer_exchange = group_both.plan()
    t2_R = evaluate_time(buffer_exchange, info="buffer_exchange_t2_R\t")

    buffer_L_state = create_robotstate(buffer_exchange)
    # Picking from buffer for left arm
    group_both.set_start_state(buffer_L_state)
    group_both.clear_pose_targets()
    group_both.set_pose_target(home_L, left_arm)
    # Planning the homing of left arm
    homing2_L = group_both.plan()
    t3_R = evaluate_time(homing2_L, info="homing_left_arm_t3_R\t")
    homing_L_state = create_robotstate(homing2_L)
    group_both.set_start_state(homing_L_state)
    # Picking the second test tube and placing the first in mean while
    (t4_R, place_R, return_home_R) = placing_both(T1, T2)
    buffer_second_tube = create_robotstate(return_home_R)
    # Picking from buffer for left arm
    group_both.set_start_state(buffer_second_tube)
    group_both.clear_pose_targets()
    group_both.set_pose_target(placePS[3], left_arm)
    group_both.set_pose_target(home_R, right_arm)
    # Planning the exchange
    buffer_exchange2 = group_both.plan()
    t5_R = evaluate_time(buffer_exchange2, info="exchange2_t5_R\t")
    placing2 = create_robotstate(buffer_exchange2)
    group_both.set_start_state(placing2)
    (t6_R, place_R2, return_home_R2) = placing(T2, "place_exchange_t6_R\t")

    # Evaluation of the time for Right arm
    duration_R = t1_R + t2_R + t3_R + t4_R + t5_R + t6_R
    rospy.loginfo('TOTAL TIME Left: {}s'.format(duration_L))
    rospy.loginfo('TOTAL TIME Right: {}s'.format(duration_R))

    if duration_R < duration_L:
        rospy.loginfo('Motion Right wins')
        # # RIGHT motion
        # Execute picking
        group_both.execute(pick_R)
        group_both.stop()
        # Attach Test tube
        T1.attach_object(right_arm)
        gripper_effort(RIGHT, 10)
        # Going to the buffer position
        group_both.execute(buffer_R)
        group_both.stop()
        # Open the gripper and detach the object
        T1.detach_object(right_arm)
        gripper_effort(RIGHT, -20)
        # buffer exchange
        group_both.execute(buffer_exchange)
        group_both.stop()
        T1.attach_object(left_arm)
        gripper_effort(LEFT, 10)
        # homing the L
        group_both.execute(homing2_L)
        group_both.stop()
        # placing
        group_both.execute(place_R)
        group_both.stop()
        T1.detach_object(left_arm)
        gripper_effort(LEFT, -20)
        T2.attach_object(right_arm)
        gripper_effort(RIGHT, 10)
        # return to home
        group_both.execute(return_home_R)
        group_both.stop()
        T2.detach_object(right_arm)
        gripper_effort(RIGHT, -20)
        # buffer exchange
        group_both.execute(buffer_exchange2)
        group_both.stop()
        # Attach object and close gripper
        T2.attach_object(left_arm)
        gripper_effort(LEFT, 10)
        group_both.execute(place_R2)
        group_both.stop()
        # Detach object and open gripper
        T2.detach_object(left_arm)
        gripper_effort(LEFT, -20)
        group_both.execute(return_home_R2)
        group_both.stop()
    else:
        rospy.loginfo('Motion Left wins')
        # # LEFT motion
        # Execute Picking
        group_both.execute(pick_L)
        group_both.stop()

        # Attach Test tube
        T1.attach_object(left_arm)
        gripper_effort(LEFT, 10)
        group_both.execute(homing_L)
        group_both.stop()

        # Execute Placing
        group_both.execute(place_L)
        group_both.stop()
        T1.detach_object(left_arm)
        gripper_effort(LEFT, -20)
        group_both.execute(return_home_L)
        group_both.stop()
        # 2nd tube
        # Execute Picking
        group_both.execute(pick2_L)
        group_both.stop()

        # Attach Test tube
        T2.attach_object(left_arm)
        gripper_effort(LEFT, 10)
        group_both.execute(homing_L2)
        group_both.stop()

        # Execute Placing
        group_both.execute(place2_L)
        group_both.stop()
        T2.detach_object(left_arm)
        gripper_effort(LEFT, -20)
        group_both.execute(return_home2_L)
        group_both.stop()

    # Representing data and saving data
    joint_diagram(pick_L, "pick L")
    joint_diagram(homing_L, "homing L")
    joint_diagram(place_L, "place L")
    joint_diagram(return_home_L, "return home L")
    joint_diagram(pick2_L, "pick2 L")
    joint_diagram(homing_L2, "homing L2")
    joint_diagram(place2_L, "place2 L")
    joint_diagram(return_home2_L, "return home2 L")

    # Saving the total time in the log file
    with open(path.join(path.expanduser("~"), "Thesis/timings"), "a+") as f:
        f.write("Total time: {}s\n".format(duration_L))
        f.write("------------------\n")


if __name__ == '__main__':
    run()
    roscpp_shutdown()
