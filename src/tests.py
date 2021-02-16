#!/usr/bin/env python

from copy import deepcopy
from moveit_msgs.msg import *
from moveit_commander import *
import yaml
import os
import geometry_msgs.msg
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


def run():
    rospy.logdebug(robot.get_group_names())
    gripper_effort(LEFT, -20)
    gripper_effort(RIGHT, -20)
    # Remove detached object
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
    home_state = robot.get_current_state()
    rospy.loginfo('current pose: \n L: {} \n R: {}'.format(
        group_both.get_current_pose(left_arm), group_both.get_current_pose(right_arm)))
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


if __name__ == '__main__':
    # Initialization of the Node
    rospy.init_node('test', anonymous=True, log_level=rospy.DEBUG)
    run()
    roscpp_shutdown()
