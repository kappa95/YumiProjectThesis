#!/usr/bin/env python

from copy import deepcopy
from typing import List
from moveit_msgs.msg import *
from moveit_commander import *
from visualization_msgs.msg import MarkerArray, Marker
from rospy_message_converter import message_converter
import tf
import yaml
from geometry_msgs.msg import *
from yumi_utils import PI, gripper_effort, LEFT, RIGHT
from yumi_hw.srv import *


# Initialization of Moveit
rospy.loginfo('Starting the Initialization')
roscpp_initialize(sys.argv)
# Initialization of the Node
rospy.init_node('test', anonymous=True, log_level=rospy.DEBUG)

robot = RobotCommander()
scene = PlanningSceneInterface()
mpr = MotionPlanRequest()
rospy.sleep(1.0)

group_both = MoveGroupCommander("fede_both")
group_both.set_pose_reference_frame("yumi_body")
group_both.set_planning_time(20)
group_both.set_num_planning_attempts(10)
group_both.allow_replanning(True)

# Arms end effectors
right_arm = "yumi_link_7_r"
left_arm = "yumi_link_7_l"
group_left_gripper = 'left_gripper'
group_right_gripper = 'right_gripper'


# Publish the trajectory on Rviz
# rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
# rospy.sleep(1.0)


# Home position
home_L = [0.300, 0.250, 0.380, 0, PI, 0]
home_R = [0.300, -0.250, 0.380, 0, PI, PI]

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


def getting_joints_from_plan(plan):
    # type: (RobotTrajectory) -> list
    """
    Provide the joints of the last point of the trajectory

    :param plan: RobotTrajectory msg: Plan of a trajectory
    :type plan: RobotTrajectory
    :rtype: list
    :return: List of joints of the end of the trajectory
    """
    plan_dict = message_converter.convert_ros_message_to_dictionary(plan)  # type: dict
    positions = plan_dict['joint_trajectory']['points'][-1]['positions']
    return positions


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


def evaluate_time(plan):
    """
    It returns the time duration of the trajectory

    :param plan: Plan msg of the trajectory
    :type plan: RobotTrajectory
    :rtype: float
    :return: duration time of the trajectory
    """
    # Check if the plan is not empty
    if plan.joint_trajectory.points:
        plan_dict = message_converter.convert_ros_message_to_dictionary(plan)  # type: dict
        duration_time = plan_dict['joint_trajectory']['points'][-1]['time_from_start']  # type: dict
        duration = int(duration_time['secs']) + int(duration_time['nsecs'])*10**(-9)  # type: float
        rospy.loginfo('Estimated time of planning: {} s'.format(duration))
        return duration
    else:
        rospy.logwarn('The plan is empty')
        duration = 0.0
        return duration


def home():
    rospy.loginfo('home')
    group_both.set_pose_target(home_L, left_arm)
    group_both.set_pose_target(home_R, right_arm)
    plan = group_both.plan()
    home_duration = evaluate_time(plan)
    group_both.execute(plan)
    group_both.stop()
    return home_duration


def picking(obj, arm):
    # type: (TestTube, str) -> List[float, RobotTrajectory, RobotTrajectory]
    """
    Wrapper for picking

    :param obj: Object to pick
    :param arm: Arm used
    :return: Duration time for picking
    """
    pose_P = deepcopy(obj.pose_msg)
    pose_P.pose.position.z += tube_length/2 + 0.12
    pose_P.pose.orientation = group_both.get_current_pose(arm).pose.orientation

    if arm is left_arm:
        home_pose = home_L
        grip = LEFT
    else:
        home_pose = home_R
        grip = RIGHT

    gripper_effort(grip, -20)
    group_both.set_pose_target(pose_P, arm)
    pick = group_both.plan()
    # Evaluate the time for picking
    t1 = evaluate_time(pick)
    if pick.joint_trajectory.points:
        # Creating a RobotState for evaluate the next trajectory
        create_robotstate(pick)
        # joint_state = JointState()
        # joint_state.header.stamp = rospy.Time.now()
        # joint_state.name = pick.joint_trajectory.joint_names
        #
        # plan_dict = message_converter.convert_ros_message_to_dictionary(pick)  # type: dict
        # positions = plan_dict['joint_trajectory']['points'][-1]['positions']
        # # rospy.logdebug(
        # #     'CHECK THE TRAJECTORY POINTS: \n {}'.format(positions))
        # joint_state.position = positions
        # robot_state = RobotState()
        # robot_state.joint_state = joint_state
        # group_both.set_start_state(robot_state)
        # # Execute Picking
        # group_both.execute(pick)
        # group_both.stop()
        #
        # # Attach Test tube
        # obj.attach_object(arm)

        # gripper_effort(grip, 10)
        group_both.set_pose_target(home_pose, arm)
        homing = group_both.plan()
        # Evaluate the duration of the planning
        t2 = evaluate_time(homing)
        # # Execute the trajectory
        # group_both.execute(plan)
        # group_both.stop()
        return [(t1 + t2), pick, homing]
    else:
        rospy.logerr('Planning failed')
        pass


def placing(obj):
    # type: (TestTube) -> float
    idx = obj.id  # Index of the test tube from the class
    group_both.set_pose_target(placePS[idx], left_arm)
    placePlan = group_both.plan()
    # Evaluate the time of the trajectory
    t1 = evaluate_time(placePlan)
    group_both.execute(placePlan)
    group_both.stop()
    obj.detach_object(left_arm)
    gripper_effort(LEFT, -20)
    group_both.set_pose_target(home_L, left_arm)
    plan = group_both.plan()
    t2 = evaluate_time(plan)
    group_both.execute(plan)
    return t1 + t2


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

    # Evaluate the time for the LEFT arm cycle
    (t, pick, homing) = picking(T1, left_arm)
    # # Execute Picking
    group_both.execute(pick)
    group_both.stop()

    # Attach Test tube
    T1.attach_object(left_arm)
    gripper_effort(LEFT, 10)
    group_both.execute(homing)
    group_both.stop()


if __name__ == '__main__':
    run()
    roscpp_shutdown()
