#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, NodeType

class ApriltagNode(DTROS):

    def __init__(self, node_name):
        super(ApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        print("nice")
        # self.img = rospy.Subscriber('chatter', CompressedImage, self.callback)

    def callback(self, data):
        print(type(data))

if __name__ == "__main__":
    tag = ApriltagNode(node_name = "apriltag_node")
    rospy.spin()