#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool


class BarcodeManager:
    """The BarcodeManager subscribe to Barcode if a String message is received Publish a Bool message"""

    def __init__(self, ):
        """Constructor for BarcodeManager"""
        self.barcode = None
        self.loop_rate = rospy.Rate(5)
        self.pub = rospy.Publisher('/barcode/response', Bool, queue_size=1)
        self.response = False
        self.counter = 0
        rospy.Subscriber('barcode/barcode', String, self.callback)

    def callback(self, msg):
        try:
            self.barcode = msg.data
            rospy.logdebug('I received: {}'.format(msg.data))
        except rospy.ROSException as err:
            rospy.logerr(err)

    def start(self):
        rospy.logdebug('start receiving')
        while not rospy.is_shutdown():
            if self.barcode:
                self.response = True
                rospy.loginfo('I received the barcode: {}'.format(self.barcode))
                # TODO: Add the part for saving a json file with barcodes position (use counter)
                self.pub.publish(self.response)
            else:
                self.response = False
                self.pub.publish(self.response)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.loginfo('BarcodeManager Started')
    rospy.init_node('BarcodeManager', log_level=rospy.DEBUG)
    node = BarcodeManager()
    node.start()
