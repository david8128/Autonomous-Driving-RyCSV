#!/usr/bin/env python

import rospy
import roslib
import math
import numpy as np
import time
from lane_detection import line_detector
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class vision_node:

    def __init__ (self):
        self.detector = line_detector(True)
        self.bridge = CvBridge()
        self.frame = np.array([])
        self.detected_frame = Image()
        self.hough_frame = Image()

        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.vision_callback)
        self.detection_pub = rospy.Publisher('vision/lines_detected', Image, queue_size=5)
        self.detection_pub = rospy.Publisher('vision/hough', Image, queue_size=5)

        rospy.init_node('Vision_Road_Node', anonymous=True)
        rospy.loginfo("Vision node init")
        self.rate = rospy.Rate(10) # 10 Hz ROS

    def vision_callback(self, data):
            rospy.loginfo("Frame recieved")
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.frame = np.copy(cv_image)

if __name__ == "__main__":

    node = vision_node()    
    rospy.wait_for_message("camera/rgb/image_raw",Image)

    while (not rospy.is_shutdown()):

        node.detector.update_from_ros(node.frame)
        node.detected_frame = node.bridge.cv2_to_imgmsg(node.detector.frame_detected, "bgr8")
        node.hough_frame = node.bridge.cv2_to_imgmsg(node.detector.frame_hough, "bgr8")
        node.detection_pub.publish(node.detected_frame)
        node.rate.sleep()