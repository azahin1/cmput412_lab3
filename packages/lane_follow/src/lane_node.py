#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

class LaneNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        botName = os.environ['HOSTNAME']

        # lane detection subscriber
        self.cameraSub = rospy.Subscriber(f"{botName}/camera_node/image/compressed", CompressedImage, self.moveWheels)

        # wheel movement publisher
        self.wheelPub = rospy.Publisher(f"{botName}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size = 1)
        self.wheelVel = [0.2, 0.2]

        rospy.on_shutdown(self.shutdown)

    def followLane(self, data): # only change the wheels speeds
        # process image data
        # error is the difference of the bots position from the left line and the right line
        # |---D---| so it stays in the center

        # change wheel speeds based on image data
        self.wheelVel[0] = 1
        self.wheelVel[1] = 1

    def moveWheels(self, data): # only move the wheels based on velocity
        wheelsMsg = WheelsCmdStamped()

        wheelsMsg.vel_left = self.wheelVel[0]
        wheelsMsg.vel_right = self.wheelVel[1]

        self.wheelPub.publish(wheelsMsg)

    def shutdown(self):
        self.wheelPub.unregister()

if __name__ == '__main__':
    # create the node
    laneNode = LaneNode(node_name='lane_node')
    try:
        # keep spinning
        rospy.spin()
    finally:
        laneNode.stopWheels()