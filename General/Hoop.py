# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 10:08:08 2021

@author: study
"""

import cv2
import numpy as np
import math
#detect hoop from an image (img)
#pass in upper and lower thresholds (numpy array) for hsv thresholding

#has attributes ellipse (a matrix of points), 
#rectangle (a list of the four corners of bounding rectangle),
#center (tuple containing the center of ellipse),
#contour (a matrix containing the max contour of the hoop)

class Hoop:
    #constants
    #objp: top, left, bottom, right, center
    objp=np.array([(0,23,0), (-23,0,0), (0,-23,0), (23,0,0), (0,0,0)], dtype='double')
    camera_matrix=np.array([[921.170702, 0.000000, 459.904354], 
                            [0.000000, 919.018377, 351.238301], 
                            [0.000000, 0.000000, 1.000000]])
    dist_coef=np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

    def __init__(self, img, lower_thresh, upper_thresh):
        hsv=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        self.mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
        kernel = np.ones((5,5),np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        self.res = cv2.bitwise_and(img, img, mask=self.mask)
     
        self.contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.seenHoop = False
        if len(self.contours)>0:
            self.contour = max(self.contours, key = cv2.contourArea)

            x,y,w,h = cv2.boundingRect(self.contour)
            self.rect=(x,y,w,h)
            if self.contour.shape[0] > 5:
                self.ellipse = cv2.fitEllipse(self.contour)
                self.area = int(self.ellipse[1][0] * self.ellipse[1][1] * math.pi)
                if self.area > 1000: #Hopefully this is the full hoop
                    self.seenHoop = True
                    self.center = (int(self.ellipse[0][0]), int(self.ellipse[0][1]))
                    self.euler, self.rvecs, self.tvecs = self.find_pos_ori()
                    self.imgpts = self.project_axes()
            else:
                self.center = (int(x+w//2), int(y+h//2))

    def find_pos_ori(self):
        #find points
        box = cv2.boxPoints(self.ellipse) 
        p1=(int((box[0,0]+box[1,0])/2), int((box[0,1]+box[1,1])/2))
        p2=(int((box[1,0]+box[2,0])/2), int((box[1,1]+box[2,1])/2))
        p3=(int((box[2,0]+box[3,0])/2), int((box[2,1]+box[3,1])/2))
        p4=(int((box[3,0]+box[0,0])/2), int((box[3,1]+box[0,1])/2))
        box=np.array([p1,p2,p3,p4])
        
        #order points
        left_point_x = np.min(box[:, 0])
        right_point_x = np.max(box[:, 0])
        top_point_y = np.min(box[:, 1])
        bottom_point_y = np.max(box[:, 1])
         
        left_point_y = box[:, 1][np.where(box[:, 0] == left_point_x)][0]
        right_point_y = box[:, 1][np.where(box[:, 0] == right_point_x)][0]
        top_point_x = box[:, 0][np.where(box[:, 1] == top_point_y)][0]
        bottom_point_x = box[:, 0][np.where(box[:, 1] == bottom_point_y)][0]
         
        # Four point coordinates up, left, down, and right (ccw)
        midpoints = np.array([(top_point_x, top_point_y),
                              (left_point_x, left_point_y), 
                              (bottom_point_x, bottom_point_y), 
                              (right_point_x, right_point_y)], dtype='double')
        
        imgpoints=np.vstack((midpoints, self.center))
        
        #Solve pose
        _, rvecs, tvecs = cv2.solvePnP(self.objp, imgpoints, Hoop.camera_matrix, self.dist_coef,flags= cv2.SOLVEPNP_ITERATIVE)

        #some more processing
        R=cv2.Rodrigues(rvecs)[0]
        # Checks if a matrix is a valid rotation matrix.
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        if n < 1e-6:
            # Calculates rotation matrix to euler angles
            # The result is the same as MATLAB except the order
            # of the euler angles ( x and z are swapped ).
            sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
            singular = sy < 1e-6
        
            if  not singular :
                x = math.atan2(R[2,1] , R[2,2])*57.2958+180
                y = math.atan2(-R[2,0], sy)*57.2958
                z = math.atan2(R[1,0], R[0,0])*57.2958
            else :
                x = math.atan2(-R[1,2], R[1,1])*57.2958+180
                y = math.atan2(-R[2,0], sy)*57.2958
                z = 0
            #tvec is in cm, order is x,y,z
            #x corresponds to tello's left/right
            #y corresponds to tello's up/down
            #z corresponds to tello's forward/back
            #rotation is in euler angles x,y,z in degrees
            return np.array([x, y, z]), rvecs, tvecs
    
    def project_axes(self, axis_length=10):
        axis = np.float32([[axis_length,0,0], [0,axis_length,0], [0,0,-axis_length]]).reshape(-1,3)
        imgpts, jac = cv2.projectPoints(axis, self.rvecs, self.tvecs, Hoop.camera_matrix, Hoop.dist_coef)
        imgpts=imgpts.astype(int)
        
        return imgpts