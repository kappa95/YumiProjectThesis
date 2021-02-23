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

robot = RobotCommander()
scene = PlanningSceneInterface()
mpr = MotionPlanRequest()
rospy.sleep(1.0)

group_both = MoveGroupCommander("fede_both")
group_both.set_pose_reference_frame("yumi_body")
group_both.set_planning_time(20)
group_both.allow_replanning(True)

# Arms end effectors
right_arm = "yumi_link_7_r"
left_arm = "yumi_link_7_l"
group_left_gripper = 'left_gripper'
group_right_gripper = 'right_gripper'

# Useful Variables
Z = 2

# Publish the trajectory on Rviz
rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
rospy.sleep(1.0)


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
    count = 1  # Counter of the instances for naming the object

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
        TestTube.count += 1
        if name is not None:
            self.name = "test_tube_" + str(name)
        else:
            self.name = "test_tube_" + str(TestTube.count)

    def add_object(self):
        scene.add_box(self.name, self.pose_msg, size=(self.radius, self.radius, self.length))

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
        self.placeMarkers = [i for i in self.msg.markers]  # type: List[Marker]


def evaluate_time(plan):
    """
    It returns the time duration of the trajectory

    :param plan: Plan msg of the trajectory
    :type plan: RobotTrajectory
    :return: duration time of the trajectory
    """
    # Check if the plan is not empty
    if plan:
        plan_dict = message_converter.convert_ros_message_to_dictionary(plan)  # type: dict
        duration_time = plan_dict['joint_trajectory']['points'][-1]['time_from_start']  # type: dict
        duration = int(duration_time['secs']) + int(duration_time['nsecs'])*10**(-9)  # type: float
        rospy.loginfo('Estimated time of planning: {} s'.format(duration))
        return duration
    else:
        rospy.logwarn('The plan is empty')
        duration = 0.0
        return duration


def run():
    # Take Place positions
    placeSub = PlaceSub()
    # Open grippers
    gripper_effort(LEFT, -20)
    gripper_effort(RIGHT, -20)
    # Remove detached object
    if scene.get_attached_objects():
        scene.remove_attached_object(left_arm)
    # Remove all the objects
    scene.remove_world_object()
    rospy.sleep(1.0)

    # Add the test tube
    T1 = TestTube()
    T1.add_object()
    T2 = TestTube(y=0.03)
    T2.add_object()
    rospy.sleep(2.0)

    rospy.loginfo('home')
    group_both.set_pose_target(home_L, left_arm)
    group_both.set_pose_target(home_R, right_arm)
    plan = group_both.plan()
    group_both.execute(plan)
    group_both.stop()

    pose_L = deepcopy(T1.pose_msg)
    pose_L.pose.position.z += tube_length/2 + 0.12
    pose_L.pose.orientation = group_both.get_current_pose(left_arm).pose.orientation

    group_both.set_pose_target(pose_L, left_arm)
    pick = group_both.plan()
    # Evaluate the time for picking
    evaluate_time(pick)
    # Picking
    group_both.execute(pick)
    group_both.stop()

    # Attach Test tube
    T1.attach_object(left_arm)

    gripper_effort(LEFT, 10)
    group_both.set_pose_target(home_L, left_arm)
    plan = group_both.plan()
    # Evaluate the duration of the planning
    evaluate_time(plan)
    # Execute the trajectory
    group_both.execute(plan)
    group_both.stop()

    placePS = []
    for i in placeSub.placeMarkers:
        temp = PoseStamped()
        temp.header.frame_id = i.header.frame_id
        temp.pose = i.pose
        temp.pose.orientation = group_both.get_current_pose(left_arm).pose.orientation
        temp.pose.position.z += tube_length/2 + 0.136 + 0.01 + 0.03002
        placePS.append(temp)

    group_both.set_pose_target(placePS[0], left_arm)
    placePlan = group_both.plan()
    # Evaluate the time of the trajectory
    evaluate_time(placePlan)
    group_both.execute(placePlan)
    group_both.stop()
    # Detach test tube
    T1.detach_object(left_arm)

    # The actual pose is read in the planning reference frame --> world one
    rospy.logdebug('Actual Pose:\n{}'.format(group_both.get_current_pose(left_arm).pose))
    tfl = tf.TransformListener()
    tfl.waitForTransform("base", "world", rospy.Time(0), rospy.Duration(5))
    pose_transformed = tfl.transformPose("world", placePS[0])
    rospy.logdebug('Commanded Pose:\n{}'.format(pose_transformed))
    rospy.logdebug(group_both.get_pose_reference_frame())

    # RETURN TO HOME
    group_both.set_pose_target(home_L, left_arm)
    plan = group_both.plan()
    # Evaluate the time of the trajectory
    evaluate_time(plan)
    group_both.execute(plan)
    group_both.stop()
    # Open Fingers
    gripper_effort(LEFT, -20)

    # Picking
    pose_L = deepcopy(T2.pose_msg)
    pose_L.pose.position.z += tube_length/2 + 0.12
    pose_L.pose.orientation = group_both.get_current_pose(left_arm).pose.orientation

    group_both.set_pose_target(pose_L, left_arm)
    pick = group_both.plan()

    # Evaluate the time of the trajectory
    evaluate_time(pick)
    # Execute trajectory
    group_both.execute(pick)
    group_both.stop()

    # Attach test tube
    T2.attach_object(left_arm)
    gripper_effort(LEFT, 10)

    # Homing
    group_both.set_pose_target(home_L, left_arm)
    plan = group_both.plan()

    # Evaluate the time of the trajectory
    evaluate_time(plan)
    # Execute trajectory
    group_both.execute(plan)
    group_both.stop()

    # Placing
    group_both.set_pose_target(placePS[1], left_arm)
    placePlan = group_both.plan()
    # Evaluate the time of the trajectory
    evaluate_time(placePlan)
    group_both.execute(placePlan)
    group_both.stop()
    T2.detach_object(left_arm)
    group_both.set_pose_target(home_L, left_arm)
    group_both.go(wait=True)


if __name__ == '__main__':
    # Initialization of the Node
    rospy.init_node('test', anonymous=True, log_level=rospy.DEBUG)
    run()
    roscpp_shutdown()
