#!/usr/bin/env python

import numpy as np
import cv2
import os
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import warnings
warnings.simplefilter('ignore', np.RankWarning)
class line_detector:

    def __init__(self, ros):
        self.image_path = os.path.abspath(os.path.join(os.path.abspath(__file__), '../../tests'))
        self.image_name = "" 
        self.frame = np.array([])
        self.frame_gray = np.array([])
        self.frame_canny = np.array([])
        self.frame_masked = np.array([])
        self.frame_hough = np.array([])
        self.frame_detected = np.array([])
        self.frame_raw_lines = np.array([])
        self.frame_mask = np.array([])
        self.heigth = 0
        self.width = 0
        self.left_line = np.array([])
        self.right_line = np.array([])
        self.ros_flag = ros
        self.center = 0
        self.canny_top = 0

    def read_frame_file(self, image_name):
        image = cv2.imread(self.image_path +'/'+ image_name)
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.frame = np.array(image)
        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        self.frame_gray = np.array(gray)
        self.heigth = image.shape[0]
        self.width = image.shape[1]  
        self.canny_top =  self.heigth/2
    
    def read_frame_ros_msgs(self, image):
        self.frame = np.array(image)
        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        self.frame_gray = np.array(gray)
        self.heigth = image.shape[0]
        self.width = image.shape[1] 
        self.canny_top =  self.heigth/2

    def display_frame(self, frame):
        cv2.imshow('Current Frame',frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def plot_all_frames(self):

        fig, ax = plt.subplots(2, 4,figsize=(8, 4))

        ax[0,0].imshow(self.frame)
        ax[0,0].set_title("Original Frame")

        ax[0,1].imshow(self.frame_gray, cmap='gray')
        ax[0,1].set_title("Grayscale")

        ax[0,2].imshow(self.frame_canny, cmap='gray')
        ax[0,2].set_title("Canny edges")

        ax[0,3].imshow(self.frame_masked, cmap='gray')
        ax[0,3].set_title("Canny croped")

        ax[1,0].imshow(self.frame_hough)
        ax[1,0].set_title("Hough Lines")
    
        ax[1,1].imshow(self.frame_raw_lines)
        ax[1,1].set_title("Average lines")

        ax[1,2].imshow(self.frame_detected )
        ax[1,2].set_title("Line detection result")

        ax[1,3].imshow(self.frame_mask)
        ax[1,3].set_title("Mascara")

        self.frame_detected = np.array([])

        fig.suptitle('Frame proccesing')

        plt.show()

    def edge_detection(self):
        blur = cv2.GaussianBlur(self.frame_gray, (5, 5), 0)  
        canny = cv2.Canny(blur, 50, 150) 
        self.frame_canny = np.array(canny)
    
    def mask_interest_region(self):
        mask_points = np.array([[0,self.heigth/2],
                                [0, self.heigth],
                                [self.width/2, self.heigth/2]], 'int32')
        mask = np.zeros_like(self.frame_canny)
        cv2.fillConvexPoly(mask, mask_points, 255)
        mask_points = np.array([[self.width,self.heigth/2],
                                [self.width, self.heigth],
                                [self.width/2, self.heigth/2]], 'int32')
        cv2.fillConvexPoly(mask, mask_points, 255)

        self.frame_mask = mask
        self.frame_masked = cv2.bitwise_and(self.frame_canny, mask)
    
    def lines_hough(self):
        lines = cv2.HoughLinesP(self.frame_masked, 1, np.pi/180,50, minLineLength=100, maxLineGap=10)
        if lines is not None: 
            temp = np.copy(self.frame)
            for line in lines:
                x1,y1,x2,y2 = line[0]
                cv2.line(temp, (x1,y1),(x2,y2),(0,255,0),2)
            self.frame_hough = temp
            """ print("Lines Detected: ")
            print(lines) """
            #print("Number of lines detected: "+str(len(lines)))
        else:
            print("No lines detected")
            self.frame_hough = np.zeros_like(self.frame)
        return lines
    
    def average_lines(self, lines):
        if lines is not None: 
            left_group = np.zeros((1,2))
            right_group = np.zeros((1,2))

            for line in lines:
                x1, y1, x2, y2 = line.reshape(4) 
                parameters = np.polyfit((x1, x2), (y1, y2), 1)  
                slope = parameters[0] 
                intercept = parameters[1] 
                if slope < 0: 
                    left_group = np.append(left_group,[[slope, intercept]],axis=0) 
                else: 
                    right_group = np.append(right_group,[[slope, intercept]],axis=0) 
            
            left_group = np.delete(left_group, 0, 0)
            right_group = np.delete(right_group, 0, 0)
            
            """ print("Left lines : ")
            print(left_group)
            print("Right lines: ")
            print(right_group) """

            left_group_average = np.average(left_group , axis = 0) 
            right_group_average = np.average(right_group , axis = 0) 

            """ print("Left lines average: "+str(left_group_average))
            print("Right lines average: "+str(right_group_average)) """

            left_line = self.create_coordinates(left_group_average) 
            right_line = self.create_coordinates(right_group_average) 

            self.left_line = left_line
            self.right_line = right_line

            #print("Left line coordinates: "+ str(left_line))
            #print("Right line coordinates: "+ str(right_line))
        else:
            self.left_line = ([0,0,0,0])
            self.right_line = ([0,0,0,0])

    
    def create_coordinates(self, line_parameters): 
        if not math.isnan(line_parameters[0]):
            slope, intercept = line_parameters 
            y1 = self.frame_gray.shape[0] 
            #y2 = int(y1*3/5) 
            y2 = self.canny_top
            if slope == 0:
                return np.array([0, 0, 0, 0]) 
            x1 = int((y1 - intercept) / slope) 
            x2 = int((y2 - intercept) / slope) 

            #Handle for overflow of int
            if x1 > 1000000:
                x1 = 1000000
            if x2 > 1000000:
                x2= 1000000
            if x1 < -1000000:
                x1 = -1000000
            if x2 < -1000000:
                x2= -1000000
            
            return np.array([x1, y1, x2, y2])  
        else:
            return np.array([0, 0, 0, 0]) 

    def draw_lines(self): 
        line_image = np.zeros_like(self.frame) 
        lines = ([self.left_line, self.right_line])
        if lines is not None: 
            for x1, y1, x2, y2 in lines: 
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 10) 
        cv2.circle(line_image,(self.center,self.canny_top), 15, (0,0,255), -1)
        cv2.line(line_image, (self.width/2, self.canny_top+30), (self.width/2, self.canny_top-30), (0, 255, ), 10)
        cv2.line(line_image, (self.center, self.canny_top+30), (self.center, self.canny_top-30), (0, 0, 255), 10)
        self.frame_raw_lines = line_image
        self.frame_detected = cv2.addWeighted(line_image, 0.8, self.frame, 1, 1) 


    def calculate_center(self):
        left_x = self.left_line[2]
        right_x = self.right_line[2]
        self.center = (left_x + right_x)/2
        #print("Aproximate center: " + str(self.center))

    
    def update_from_file(self):
        self.read_frame_file(self.image_name)
        self.edge_detection()
        self.mask_interest_region()
        lines = self.lines_hough()
        self.average_lines(lines)
        self.draw_lines()
 
    def update_from_ros(self, ros_img):
        self.read_frame_ros_msgs(ros_img)
        self.edge_detection()
        self.mask_interest_region()
        lines = self.lines_hough()
        self.average_lines(lines)
        self.calculate_center()
        self.draw_lines()
 
#Test from saved IMG's
""" if __name__ == "__main__":

    detector = line_detector(False)
    detector.image_name = "road_14.png"
    detector.update_from_file()
    detector.plot_all_frames() """


    

    
  