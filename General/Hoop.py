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
        self.mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
        kernel = np.ones((5,5),np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        self.res = cv2.bitwise_and(img, img, mask=self.mask)
    
        
        self.contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(self.contours)>0:
            self.contour = max(self.contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(self.contour)
            self.rect=(x,y,w,h)
            if self.contour.shape[0] > 5:
                self.ellipse = cv2.fitEllipse(self.contour)
                self.center = (int(self.ellipse[0][0]), int(self.ellipse[0][1]))
            else:
                self.center = (int(x+w//2), int(y+h//2))
        




        
        
        
    
    