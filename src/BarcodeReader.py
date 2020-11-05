#! /usr/bin/env python

import rospy
import cv2
import zbar
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Nodo:
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.pub = rospy.Publisher('Barcode', String, queue_size=1)
        self.Scanner = zbar.Scanner()
        # self.window = cv2.namedWindow('output', cv2.WINDOW_NORMAL)
        rospy.Subscriber('yumi/right_cam_image', Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Image Received')
        self.image = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')

    def start(self):
        rospy.loginfo('Start reading')
        while not rospy.is_shutdown():
            if self.image is not None:
                rospy.loginfo('Scanning of the barcode')
                results = self.Scanner.scan(self.image)
                if results is not None:
                    for results in results:
                        barcode = results.data
                        if barcode is not None:
                            rospy.loginfo(barcode)
                            rospy.loginfo('Publish of the barcode')
                            self.pub.publish(barcode)
                            exit(200)
                else:
                    rospy.logerr('No barcodes readed')
            else:
                rospy.logerr('No Images Readed')
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.loginfo('Inizializzazione del nodo')
    rospy.init_node('BarcodeReader', anonymous=False)
    node = Nodo()
    node.start()
