#!/usr/bin/env python


# Import the required libraries 
import cv2 
import numpy as np 
import matplotlib.pyplot as plt 

    
def canny_edge_detector(image):   
    # Convert the image color to grayscale 
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  
      
    # Reduce noise from the image 
    blur = cv2.GaussianBlur(gray_image, (5, 5), 0)  
    canny = cv2.Canny(blur, 50, 150) 
    return canny 


def region_of_interest(image): 
    height = image.shape[0] 
    width = image.shape[1] 
    polygons = np.array([ 
        [(0, height), (width, height), (width/2, height/2)] 
        ]) 
    mask = np.zeros_like(image) 
      
    # Fill poly-function deals with multiple polygon 
    cv2.fillPoly(mask, polygons, 255)  
      
    # Bitwise operation between canny image and mask image 
    masked_image = cv2.bitwise_and(image, mask)  
    return masked_image 


def create_coordinates(image, line_parameters): 
    slope, intercept = line_parameters 
    y1 = image.shape[0] 
    y2 = int(y1 * (3 / 5)) 
    x1 = int((y1 - intercept) / slope) 
    x2 = int((y2 - intercept) / slope) 
    return np.array([x1, y1, x2, y2]) 


def average_slope_intercept(image, lines): 
    left_fit = [] 
    right_fit = [] 
    for line in lines: 
        x1, y1, x2, y2 = line.reshape(4) 
          
        # It will fit the polynomial and the intercept and slope 
        parameters = np.polyfit((x1, x2), (y1, y2), 1)  
        slope = parameters[0] 
        intercept = parameters[1] 
        if slope < 0: 
            left_fit.append((slope, intercept)) 
        else: 
            right_fit.append((slope, intercept)) 
              
    left_fit_average = np.average(left_fit) 
    right_fit_average = np.average(right_fit) 
    left_line = create_coordinates(image, left_fit_average) 
    right_line = create_coordinates(image, right_fit_average) 
    return np.array([left_line, right_line]) 


def display_lines(image, lines): 
    line_image = np.zeros_like(image) 
    print(lines)
    if lines is not None: 
        for x1, y1, x2, y2 in lines: 
            print(x1)
            print(y1)
            print(x2)
            print(y2)
            cv2.line(line_image, (x1, y1), (x2, y2), (200, 10, 10), 5) 
    return line_image 


if __name__ == "__main__":
    img = cv2.imread('road_10.png')
    im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    canny_image = canny_edge_detector(im_rgb) 
    cropped_image = region_of_interest(canny_image) 
      
    cv2.imshow('image',cropped_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100,  
                            np.array([]), minLineLength = 40,  
                            maxLineGap = 5)  
    print(lines)
    averaged_lines = average_slope_intercept(im_rgb, lines)  
    line_image = display_lines(im_rgb, averaged_lines) 
    combo_image = cv2.addWeighted(im_rgb, 0.8, line_image, 1, 1)  
    cv2.imshow("results", combo_image) 
      
    # When the below two will be true and will press the 'q' on 
    # our keyboard, we will break out from the loop 
      
    # # wait 0 will wait for infinitely between each frames.  
    # 1ms will wait for the specified time only between each frames 
    if cv2.waitKey(1) & 0xFF == ord('q'):       
        
        # close the video file 
        cap.release()  
        
        # destroy all the windows that is currently on 
        cv2.destroyAllWindows()  