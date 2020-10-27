#!/usr/bin/env python

import rospy
import yumi_utils as yumi
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class Nodo(object):
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)

        self.pub = rospy.Publisher('imagetimer', Image, queue_size=10)
        rospy.Subscriber("/yumi/right_cam_image", Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo('Timing images')
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            if self.image is not None:
                cv_image = self.br.cv2_to_imgmsg(self.image)
                self.pub.publish(cv_image)
                rospy.loginfo("bubusettete")
                cv2.imshow(cv_image, "bgr8")
            else:
                rospy.loginfo('boh')
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('imagetimer', anonymous=True)
    my_node = Nodo()
    my_node.start()
