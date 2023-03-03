#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge

class LaneNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        botName = os.environ['HOSTNAME']
        self.bridge = CvBridge()

        # lane detection subscriber
        self.cameraSub = rospy.Subscriber(f"{botName}/camera_node/image/compressed", CompressedImage, self.followLane)
        self.cameraPub = rospy.Publisher(f"{botName}/camera_node/lane_tracker/compressed", CompressedImage, queue_size = 1)

        # wheel movement publisher
        self.wheelPub = rospy.Publisher(f"{botName}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size = 1)

    def followLane(self, data): # only change the wheels speeds
        # process image data
        # error is the difference of the bots position from the left line and the right line
        # |---D---| so it stays in the center

        # change wheel speeds based on image data
        img = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
        # wheelsMsg = WheelsCmdStamped()

        contours = self.makeContour(img, np.array([20, 100, 100]), np.array([30, 255, 255])) # yellow contours
        error = self.getError(img.shape, contours)

        conImg = cv2.drawContours(np.zeros_like(img), contours, -1, (30, 255, 255), 2)

        # accScaling = 0.01
        # wheelsMsg.vel_left = 1.0 + accScaling*err
        # wheelsMsg.vel_right = 1.0 - accScaling*err

        self.cameraPub.publish(self.bridge.cv2_to_compressed_imgmsg(conImg))
        # self.wheelPub.publish(wheelsMsg)

    def makeContour(self, imgData, colMin, colMax):
        hsv = cv2.cvtColor(imgData, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colMin, colMax)
        colour = cv2.bitwise_and(imgData, imgData, mask = mask)

        gray = cv2.cvtColor(colour, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        return contours
    
    def getError(self, dim, contours):
        err = 0
        w = 0
        for c in contours:
            m = cv2.moments(c)
            if m["m00"]:
                cx = int(m["m10"]/m["m00"]) # x position of the contour centroid center
                cy = int(m["m01"]/m["m00"]) # y position of the contour centroid center

                cw = cy/dim[0] if cy >= dim[0]/2 else 0 # weight of the pixels are determained by how low it is on the pic
                err += cx*cw
                w += cw
        err = (err/w - dim[1]/2) if w else 0 # if no yellow detected, just go forward
        print(f"Error: {err}")
        return err

if __name__ == '__main__':
    # create the node
    laneNode = LaneNode(node_name='lane_node')
    try:
        # keep spinning
        rospy.spin()
    finally:
        pass