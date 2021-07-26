# -*- coding: utf-8 -*-
"""
Created on Sun Jul 25 20:36:46 2021

@author: study
"""

import numpy as np
import cv2
from djitellopy import Tello
"""
import os, os.path

def get_images(path):
    imgs = []
    valid_images = [".jpg",".gif",".png",".tga"]
    for f in os.listdir(path):
        ext = os.path.splitext(f)[1]
        if ext.lower() not in valid_images:
            continue
        imgs.append(os.path.join(path,f))
    return imgs

images=get_images('./pictureDataset/green/')

images=[cv2.imread(i) for i in images]
"""

lower_blue = np.array([0, 30, 30])
upper_blue = np.array([50, 255, 255])

lower_orange = np.array([110, 20, 70])
upper_orange = np.array([125, 255, 230])

lower_green = np.array([70, 50, 70])
upper_green = np.array([95, 255, 255])

kernel = np.ones((5,5),np.uint8)

tello=Tello()
tello.connect()
tello.streamon()

#for i in range(5):
while True:
    #area=[]
    print('---------------------------------------')
    
    #hsv = cv2.cvtColor(images[i], cv2.COLOR_RGB2HSV)
    img=tello.get_frame_read().frame
    hsv=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    res = cv2.bitwise_and(img, img, mask=mask)

    grayscale = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY);
    grayscale = cv2.dilate(grayscale,kernel,iterations = 1)
    
    contours, hierarchy = cv2.findContours(grayscale, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours=np.array(contours, dtype=object)
    
    n=contours.shape
    print(n[0])
    
    c = max(contours, key = cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)
    ellipse = cv2.fitEllipse(c)
    center = (int(ellipse[0][0]), int(ellipse[0][1]))
    print(center)
    cv2.drawContours(res, c, -1, (255, 0, 0), 2)
    cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),5)
    cv2.circle(res, center, 5, (255, 0, 0), -1)
    cv2.ellipse(res, ellipse, (0, 255, 0), 5)

    cv2.imshow('pic', res)
    cv2.waitKey(1)


