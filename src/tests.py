#!/usr/bin/env python

from copy import deepcopy
from typing import List
from moveit_msgs.msg import *
from moveit_commander import *
from visualization_msgs.msg import MarkerArray, Marker
import tf
import yaml
import os
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


class TestTube(name=None, x=0.0, y=0.0):
    # type: (str, float, float)
    """
    Defines the object test tube
    """
    count = 1  # Counter of the instances for naming the object

    def __init__(self, name, x, y):
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

    def detach_object(self, arm):
        # type: (TestTube, str) -> None
        scene.remove_attached_object(arm, self.name)


# Place positions are subscribed by the placepos topic
class PlaceSub:
    def __init__(self):
        self.placeMarkers = []  # type: List[Pose]
        self.msg = rospy.wait_for_message("/placepos", topic_type=MarkerArray)  # type: MarkerArray
        self.placeMarkers = [i for i in self.msg.markers]  # type: List[Marker]


def run():
    # Take Place positions
    placeSub = PlaceSub()
    gripper_effort(LEFT, -20)
    gripper_effort(RIGHT, -20)
    # Remove detached object
    if scene.get_attached_objects():
        scene.remove_attached_object(left_arm, "test_tube")
    # Remove all the objects
    scene.remove_world_object("test_tube")
    rospy.sleep(1.0)
    # Add the test tube
    scene.add_box("test_tube", test_tube_pose, size=(tube_radius, tube_radius, tube_length))
    rospy.sleep(2.0)

    rospy.loginfo('home')
    group_both.set_pose_target(home_L, left_arm)
    group_both.set_pose_target(home_R, right_arm)
    plan = group_both.plan()
    group_both.execute(plan)
    group_both.stop()
    # rospy.loginfo('current pose: \n L: {} \n R: {}'.format(
    #     group_both.get_current_pose(left_arm), group_both.get_current_pose(right_arm)))
    # Target Poses
    pose_L = deepcopy(test_tube_pose)
    pose_L.pose.position.z += tube_length/2 + 0.12
    pose_L.pose.orientation = group_both.get_current_pose(left_arm).pose.orientation
    group_both.set_pose_target(pose_L, left_arm)
    pick = group_both.plan()
    group_both.execute(pick)
    touch_links = robot.get_link_names(group_left_gripper)
    scene.attach_box(left_arm, "test_tube", touch_links=touch_links)
    gripper_effort(LEFT, 10)
    group_both.set_pose_target(home_L, left_arm)
    plan = group_both.plan()
    group_both.execute(plan)
    group_both.stop()

    # placePos = [i.pose for i in placeSub.placeMarkers]  # type: List[Pose]
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
    group_both.execute(placePlan)
    group_both.stop()
    # Detach test tube
    scene.remove_attached_object(left_arm, "test_tube")
    gripper_effort(LEFT, -20)
    # The actual pose is read in the planning reference frame --> world one
    rospy.logdebug('Actual Pose:\n{}'.format(group_both.get_current_pose(left_arm).pose))
    tfl = tf.TransformListener()
    tfl.waitForTransform("base", "world", rospy.Time(0), rospy.Duration(5))
    pose_transformed = tfl.transformPose("world", placePS[0])
    rospy.logdebug('Commanded Pose:\n{}'.format(pose_transformed))
    rospy.logdebug(group_both.get_pose_reference_frame())


if __name__ == '__main__':
    # Initialization of the Node
    rospy.init_node('test', anonymous=True, log_level=rospy.DEBUG)
    run()
    roscpp_shutdown()
