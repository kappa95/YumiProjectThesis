#!/usr/bin/env python

import copy
from typing import Any
from moveit_msgs.msg import *
from moveit_commander import *
import geometry_msgs.msg
from yumi_utils import PI, gripper_effort
from yumi_hw.srv import *

planner = "RRTConnectConfigDefault"  # Tree-based planner


class YumiWrapper:

    def __init__(self):
        self.group = {'left': MoveGroupCommander("left_arm"),
                      'right': MoveGroupCommander("right_arm"),
                      'both': MoveGroupCommander("both_arms")}
        self.robot = RobotCommander()
        self.group['left'].set_planner_id(planner)
        self.group['right'].set_planner_id(planner)
        self.group['both'].set_planner_id(planner)

        self.group['left'].set_pose_reference_frame("yumi_body")
        self.group['right'].set_pose_reference_frame("yumi_body")
        self.group['both'].set_pose_reference_frame("yumi_body")

        self.group['left'].allow_replanning(True)
        self.group['right'].allow_replanning(True)
        self.group['both'].allow_replanning(True)

        self.group['left'].set_num_planning_attempts(500)
        self.group['right'].set_num_planning_attempts(500)
        self.group['both'].set_num_planning_attempts(500)

    def cartesian(self, dest_pose, arm, constraint=None):
        # type: (geometry_msgs.msg.Pose , str, Any) -> None
        """
        Create a cartesian path with 4 points
        :param dest_pose: Pose
        :param arm: str
        :param constraint: Path Constraint
        :return: Nothing
        """
        if arm != 'left' or arm != 'right':
            raise ValueError("unknown arm: {}".format(arm))
        waypoints = []
        # Initialize the waypoints: Pose object
        wpose = copy.deepcopy(self.group[arm].get_current_pose().pose)
        # Initialize start pose: Pose object
        start_pose = copy.deepcopy(self.group[arm].get_current_pose().pose)
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
        while fraction < 1.0 and attempts < 500:
            attempts += 1
            (plan, fraction) = self.group[arm].compute_cartesian_path(
                waypoints,
                0.01,  # eef step: 1cm
                jump_threshold=0.0,
                avoid_collisions=True,
                path_constraints=constraint)
            rospy.logdebug('attempts: {} fraction: {}%'.format(attempts, fraction * 100))
        # ricalcolare il time della traiettoria!
        if fraction == 1.0:
            plan = self.group[arm].retime_trajectory(self.robot.get_current_state(), plan, 1.0)
            rospy.loginfo('executing the plan')
            self.group[arm].execute(plan, wait=True)
            self.group[arm].stop()
        else:
            rospy.logerr('it doesn\'t complete the trajectory, fraction: {}%'.format(fraction * 100))
            raise Exception('Exceeded the maximum number of retries')
