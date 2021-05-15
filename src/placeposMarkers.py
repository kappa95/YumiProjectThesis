#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from copy import deepcopy


base_height = 1.0
# Place positions
placeArray = MarkerArray()
placeMarker1 = Marker()
placeMarker1.header.frame_id = "base"
# Type corresponding to a point
placeMarker1.type = Marker.SPHERE
# In order to have the marker forever
placeMarker1.lifetime.is_zero()
placeMarker1.id = 0
placeMarker1.color.r = 1.0
placeMarker1.color.g = 0.0
placeMarker1.color.b = 0.0
placeMarker1.color.a = 1.0
placeMarker1.scale.x = 0.01
placeMarker1.scale.y = 0.01
placeMarker1.scale.z = 0.01
placeMarker1.pose.position.x = 0.0
placeMarker1.pose.position.y = 0.3
placeMarker1.pose.position.z = base_height/2
placeArray.markers.append(placeMarker1)
placeMarker2 = deepcopy(placeMarker1)
# append the placeMarker into the place_array
placeMarker2.id = 1
placeMarker2.pose.position.y = 0.35
# append the placeMarker into the place_array
placeArray.markers.append(placeMarker2)

# Create the buffer markers
bufferMarker1 = deepcopy(placeMarker1)
bufferMarker1.id = 2
bufferMarker1.color.r = 0.0
bufferMarker1.color.g = 1.0
bufferMarker1.pose.position.x = -0.1
bufferMarker1.pose.position.y = -0.025

bufferMarker2 = deepcopy(bufferMarker1)
bufferMarker2.id += 1
bufferMarker2.pose.position.y += 0.05

placeArray.markers.append(bufferMarker1)
placeArray.markers.append(bufferMarker2)


marker_publisher = rospy.Publisher("/placepos", MarkerArray, queue_size=20)

if __name__ == '__main__':
    rospy.init_node('placeposMarkers', anonymous=False, log_level=rospy.DEBUG)
    rospy.loginfo('Place and Buffer Markers are setting up')
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker_publisher.publish(placeArray)
        # rospy.spin()
        r.sleep()
