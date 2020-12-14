#!/usr/bin/env python

import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class line_detector:

    def __init__(self):
        self.image_path = os.path.abspath(os.path.join(os.path.abspath(__file__), '../../tests'))
        self.frame = np.array([])
        self.frame_gray = np.array([])
        self.frame_canny = np.array([])
        self.frame_masked = np.array([])
        self.frame_hough = np.array([])
        self.heigth = 0
        self.width = 0     

    def read_frame(self, image_name):
        image = cv2.imread(self.image_path +'/'+ image_name)
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.frame = np.array(image)
        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        self.frame_gray = np.array(gray)
        self.heigth = image.shape[0]
        self.width = image.shape[1]   

    def display_frame(self, frame):
        cv2.imshow('Current Frame',frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def plot_all_frames(self):

        fig, ax = plt.subplots(2, 2,figsize=(10, 10))

        ax[0,0].imshow(self.frame)
        ax[0,0].set_title("Original Frame")

        ax[0,1].imshow(self.frame_gray, cmap='gray')
        ax[0,1].set_title("Grayscale")

        ax[1,0].imshow(self.frame_canny, cmap='gray')
        ax[1,0].set_title("Canny edges")

        ax[1,1].imshow(self.frame_masked, cmap='gray')
        ax[1,1].set_title("Canny croped")

        fig.suptitle('Frame proccesing')

        plt.show()

    def edge_detection(self):
        blur = cv2.GaussianBlur(self.frame_gray, (5, 5), 0)  
        canny = cv2.Canny(blur, 50, 150) 
        self.frame_canny = np.array(canny)
    
    def mask_interest_region(self):
        mask_points = np.array([[0,self.heigth],
                                [0, 220],
                                [self.width, 220],
                                [self.width, self.heigth]], 'int32')
        mask = np.zeros_like(self.frame_canny)
        cv2.fillConvexPoly(mask, mask_points, 255)
        self.frame_masked = cv2.bitwise_and(self.frame_canny, mask)
    
    def line_hough(self):
        lines = cv2.HoughLinesP(self.frame_masked, 1, np.pi/180,100, minLineLength=20, maxLineGap=5)
        temp = np.copy(self.frame)
        for line in lines:
            x1,y1,x2,y2 = line[0]
            cv2.line(temp, (x1,y1),(x2,y2),(0,255,0),2)
        self.frame_hough = temp
        self.frame_hough = cv2.cvtColor(self.frame_hough,cv2.COLOR_RGB2BGR)
        
if __name__ == "__main__":

    detector = line_detector()
    detector.read_frame('road_5.png')
    detector.edge_detection()
    detector.mask_interest_region()
    detector.line_hough()
    detector.display_frame(detector.frame_hough)
    
  