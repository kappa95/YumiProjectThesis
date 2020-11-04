#! /usr/bin/env python


import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import ftplib
from contextlib import closing


bridge = CvBridge()
ftp_client = '192.168.125.201'
ftp_user = 'right_cam'
ftp_pwd = 'yumiPC'
filename = 'image.bmp'
local_filename = '~/' + filename
pixels_encoding = "bgr8"


def ftp_connection():
    with closing(ftplib.FTP(ftp_client)) as ftp:
        try:
            ftp.login(ftp_user, ftp_pwd)
            ftp.retrbinary("RETR " + filename, open(filename, 'wb').write)
        except ftplib.all_errors as e:
            rospy.loginfo('FTP errors: ')
            rospy.logerr(e)


def main():
    rospy.init_node('YumiCamerasNode', anonymous=False)
    rospy.loginfo('Initialized the YumiCamerasNode')
    right_cam_pub = rospy.Publisher("/yumi/right_cam_image", Image, queue_size=1)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        ftp_connection()
        try:
            im = cv2.imread(filename)
            right_cam_pub.publish(bridge.cv2_to_imgmsg(im, pixels_encoding))
        except CvBridgeError as cve:
            rospy.loginfo('CVBridgeError:')
            rospy.logerr(cve)
        rate.sleep()


if __name__ == '__main__':
    main()
