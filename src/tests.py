#!/usr/bin/env python


from moveit_msgs.msg import *
from moveit_commander import *
import yaml
import os
from std_msgs.msg import Bool
import geometry_msgs.msg
from yumi_utils import PI, gripper_effort
from yumi_hw.srv import *

# Initialization of the Node
rospy.init_node('test', anonymous=True, log_level=rospy.DEBUG)
# Initialization of Moveit
rospy.loginfo('Starting the Initialization')
roscpp_initialize(sys.argv)

robot = RobotCommander()
scene = PlanningSceneInterface()
mpr = MotionPlanRequest()
rospy.sleep(1.0)

group_both = MoveGroupCommander("fede_both")
group_both.set_planning_time(5)
group_both.allow_replanning(True)

# Publish the trajectory on Rviz
rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
rospy.sleep(1.0)

home_L = [0.300, 0.250, 0.380, 0, PI, 0]
p1_L = [0.320, 0.250, 0.380, 0, PI, 0]
p2_L = [0.340, 0.350, 0.400, 0, PI, 0]
p3_L = [0.450, 0.350, 0.380, 0, PI, 0]
p4_L = [0.380, 0.350, 0.400, 0, PI, 0]
poses_L = [p1_L, p2_L, p3_L, p4_L]
home_R = [0.300, -0.250, 0.380, 0, PI, PI]
p1_R = [0.320, -0.250, 0.280, 0, PI, PI]
p2_R = [0.480, -0.350, 0.380, 0, PI, PI]
p3_R = [0.360, -0.250, 0.480, 0, PI, PI]
p4_R = [0.450, -0.350, 0.380, 0, PI, PI]
poses_R = [p1_R, p2_R, p3_R, p4_R]
rospy.loginfo('random')
group_both.set_random_target()
rand = group_both.plan()
group_both.execute(rand)
group_both.stop()
rospy.loginfo('home')
group_both.set_pose_target(home_L, "yumi_link_7_l")
group_both.set_pose_target(home_R, "yumi_link_7_r")
plan = group_both.plan()
group_both.execute(plan)
group_both.stop()
rospy.loginfo('current pose: \n L: {} \n R: {}'.format(
    group_both.get_current_pose("yumi_link_7_l"), group_both.get_current_pose("yumi_link_7_r")))
rospy.loginfo('multi')
# non portano all'ultimo punto
group_both.set_pose_targets(poses_L, "yumi_link_7_l")
group_both.set_pose_targets(poses_R, "yumi_link_7_r")
# portano all'ultimo
group_both.set_pose_target(p1_L, "yumi_link_7_l")
group_both.set_pose_target(p2_L, "yumi_link_7_l")
group_both.set_pose_target(p3_L, "yumi_link_7_l")
group_both.set_pose_target(p4_L, "yumi_link_7_l")
group_both.set_pose_target(p1_R, "yumi_link_7_r")
group_both.set_pose_target(p2_R, "yumi_link_7_r")
group_both.set_pose_target(p3_R, "yumi_link_7_r")
group_both.set_pose_target(p4_R, "yumi_link_7_r")
plan_long = group_both.plan()
group_both.execute(plan_long)
rospy.loginfo('current pose: \n L: {} \n R: {}'.format(
    group_both.get_current_pose("yumi_link_7_l"), group_both.get_current_pose("yumi_link_7_r")))
file_path = os.path.join(os.path.expanduser('~'), 'plan_msg.yaml')
with open(file_path, 'w') as f:
    yaml.dump(plan_long, f, default_flow_style=True)
roscpp_shutdown()
