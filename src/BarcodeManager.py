#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool


class BarcodeManager:
    """The BarcodeManager subscribe to Barcode if a String message is received Publish a Bool message"""

    def __init__(self, ):
        """Constructor for BarcodeManager"""
        self.barcode = None
        self.loop_rate = rospy.Rate(100)
        self.pub = rospy.Publisher('/barcode/response', Bool, queue_size=1)
        self.counter = 0
        rospy.Subscriber('Barcode', String, self.callback)

    def callback(self, msg):
        try:
            self.barcode = msg
            rospy.logdebug('I received: {}'.format(msg))
        except rospy.ROSException as err:
            rospy.logerr(err)

    def start(self):
        rospy.logdebug('start receiving')
        while not rospy.is_shutdown():
            if self.barcode is not None:
                rospy.loginfo('I received the barcode: {}'.format(self.barcode))
                # TODO: Add the part for saving a json file with barcodes position (use counter)
                self.pub.publish(True)
            else:
                self.pub.publish(False)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.loginfo('BarcodeManager Started')
    rospy.init_node('BarcodeManager')
    node = BarcodeManager()
    node.start()
