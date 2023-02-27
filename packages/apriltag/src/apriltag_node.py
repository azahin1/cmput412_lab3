#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

class ApriltagNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        botName = os.environ['HOSTNAME']
        self.imgSub = rospy.Subscriber(f"{botName}/camera_node/image/compressed", CompressedImage, self.imgProcess)

    def imgProcess(self, data):
        rospy.loginfo(f"Image size: {data.format}")

if __name__ == '__main__':
    # create the node
    imgNode = ApriltagNode(node_name='camera_node')
    # keep spinning
    rospy.spin()