#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, NodeType
from dt_apriltags import Detector
import os

class AprilTagDetector(DTROS):
    def __init__(self):
        super(AprilTagDetector, self).__init__(node_name="apriltag_detector_node", node_type=NodeType.PERCEPTION)

        #print("[INFO] loading detector...")
        self.bridge = CvBridge()
        self.detector = Detector(families="tag36h11")
        self.botName = os.environ['VEHICLE_NAME']
        """
        None: White
        UofA[93,94,200,201]: Green
        T[58,62,133,153]: Blue
        Stop[162,169]: Red
        """
        # self.UofA = [93,94,200,201,143]
        # self.T = [58,62,133,153]
        # self.Stop = [162,169]
        self.count = 0

        self.colours = {
            93: "green",
            94: "green",
            200: "green",
            201: "green",
            143: "green",
            58: "blue",
            62: "blue",
            133: "blue",
            153: "blue",
            162: "red",
            169: "red",
        }

        # self.r = rospy.Rate(1)
        #while not rospy.is_shutdown():
        self.image_sub = rospy.Subscriber(f"/{self.botName}/camera_node/image/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher(f"/{self.botName}/apriltags/image/compressed", CompressedImage, queue_size=1)
            #self.r.sleep()        

    def image_callback(self, data):
        #print("[INFO] detecting tags...")
        if self.count%300:
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                return

            grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            rospy.wait_for_service('/' + self.botName + '/led_emitter_node/set_custom_pattern')
            self.service = rospy.ServiceProxy('/' + self.botName + '/led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            results = self.detector.detect(grey)

            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))

                cv2.line(cv_image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(cv_image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(cv_image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(cv_image, ptD, ptA, (0, 255, 0), 2)

                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

                tagFamily = r.tag_family.decode("utf-8")
                cv2.putText(cv_image, str(r.tag_id), (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print(f"[INFO] tag family: {r.tag_id}")

                r = results[0]
                self.change_color(self.colours[r.tag_id] if r.tag_id in self.colours else "white")

            if not results:
                self.change_color("white")

            img_msg = CompressedImage()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.format = "jpg"
            img_msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            self.image_pub.publish(img_msg)

        self.count += 1
    
    def change_color(self, color):
        msg = LEDPattern()
        msg.color_list = [color] * 5
        msg.color_mask = [1, 1, 1, 1, 1]
        msg.frequency = 0
        msg.frequency_mask = [0, 0, 0, 0, 0]
        self.service(msg)
    
if __name__ == '__main__':
    node = AprilTagDetector()
    rospy.spin()