# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 10:08:08 2021

@author: study
"""
#detect hoop from an image (img)
#pass in upper and lower thresholds (numpy array) for hsv thresholding

#has attributes ellipse (a matrix of points), 
#rectangle (a list of the four corners of bounding rectangle),
#center (tuple containing the center of ellipse),
#contour (a matrix containing the max contour of the hoop)
class Hoop:
    
    def __init__(self, img, lower_thresh, upper_thresh):
        import cv2
        import numpy as np
        
        hsv=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        res = cv2.bitwise_and(img, img, mask=mask)
    
        grayscale = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY);
        grayscale = cv2.dilate(grayscale,kernel,iterations = 1)
        
        contours, hierarchy = cv2.findContours(grayscale, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.contour = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(self.contour)
        self.rectangle=[x,y,x+w,y+h]
        self.ellipse = cv2.fitEllipse(self.contour)
        self.center = (int(self.ellipse[0][0]), int(self.ellipse[0][1]))
        




        
        
        
    
    