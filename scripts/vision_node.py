#!/usr/bin/env python

import rospy
import roslib
import math
import numpy as np
import time
import tf_conversions
import tf2_ros
from lane_detection import line_detector
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32 


class vision_node:

    def __init__ (self):
        self.detector = line_detector(True)
        self.bridge = CvBridge()
        self.frame = np.array([])
        self.detected_frame = Image()
        self.hough_frame = Image()

        self.goal_x = 0
        self.goal_y = 0
        self.goal_th = 0

        #Goal transform broadcaster
        self.goal_br = tf2_ros.TransformBroadcaster()

        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.vision_callback)
        self.detection_pub = rospy.Publisher('vision/lines_detected', Image, queue_size=5)
        self.hough_pub = rospy.Publisher('vision/hough', Image, queue_size=5)
        self.error_vision_pub = rospy.Publisher('vision/error', Int32, queue_size=10)


        rospy.init_node('Vision_Road_Node', anonymous=True)
        rospy.loginfo("Vision node init")
        self.rate = rospy.Rate(30) # 10 Hz ROS

    def vision_callback(self, data):
            #rospy.loginfo("Frame recieved")
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.frame = np.copy(cv_image)

if __name__ == "__main__":

    print("VISION WAITING FOR GAZEBO")
    rospy.wait_for_service('/gazebo/spawn_urdf_model') #Wait for model spawn
    time.sleep(2)

    node = vision_node()    
    rospy.wait_for_message("camera/rgb/image_raw",Image)
    cam_width = node.detector.width
    cam_heigth = node.detector.heigth

    while (not rospy.is_shutdown()):

        #Vision
        node.detector.update_from_ros(node.frame)
        node.detected_frame = node.bridge.cv2_to_imgmsg(node.detector.frame_detected, "rgb8")
        node.hough_frame = node.bridge.cv2_to_imgmsg(node.detector.frame_hough, "rgb8")
        node.detection_pub.publish(node.detected_frame)
        node.hough_pub.publish(node.hough_frame)
        #print("Camera width: "+str(node.detector.width))  Camera width --> 1920
        #print("Camera heigth: "+str(node.detector.heigth))  Camera heigth --> 1080
       
        #Error
        current_center = node.detector.center
        error = current_center - (node.detector.width/2)
        error_msgs = Int32()
        error_msgs.data = error
        node.error_vision_pub.publish(error_msgs)
        
        node.rate.sleep()

