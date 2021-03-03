#! /usr/bin/env python

"""Most complete way to have the IK solution - Could be easily converted into a class"""

import rospy
from moveit_commander import RobotCommander
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

right_arm = "yumi_link_7_r"
group = "fede_both"
robot = RobotCommander()

# point = PoseStamped()
# point.pose.position.x = 0.300
# point.pose.position.y = 0.300
# point.pose.position.z = 0.380


def get_ik():
    rospy.wait_for_service('compute_ik')
    # It is not necessary for a client init a node but let's do that
    rospy.init_node('query_iks')

    # Create the function for call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    while not rospy.is_shutdown():
        raw_input('Press enter to compute an IK solution:')

        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = group
        # Esempio solo per il right arm
        request.ik_request.ik_link_name = right_arm
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "yumi_body"
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state = robot.get_current_state()
        # Ask user input for Pose to reach
        x = raw_input('Write the x coordinate in meters: ')
        y = raw_input('Write the y coordinate in meters: ')
        z = raw_input('Write the z coordinate in meters: ')

        roll = raw_input('Write the r coordinate: ')
        pitch = raw_input('Write the p coordinate: ')
        yaw = raw_input('Write the y coordinate: ')

        # Conversion Euler to quaternion
        quaternion = quaternion_from_euler(float(roll), float(pitch), float(yaw))

        # Filling the PoseStamped message
        request.ik_request.pose_stamped.pose.position.x = float(x)
        request.ik_request.pose_stamped.pose.position.y = float(y)
        request.ik_request.pose_stamped.pose.position.z = float(z)

        request.ik_request.pose_stamped.pose.orientation.x = quaternion[0]
        request.ik_request.pose_stamped.pose.orientation.y = quaternion[1]
        request.ik_request.pose_stamped.pose.orientation.z = quaternion[2]
        request.ik_request.pose_stamped.pose.orientation.w = quaternion[3]

        try:
            # Send the request
            response = compute_ik(request)  # type: GetPositionIKResponse

            # print response
            print(response)

        except rospy.ServiceException as e:
            print('Rospy call failure: {}'.format(e))


if __name__ == '__main__':
    get_ik()
