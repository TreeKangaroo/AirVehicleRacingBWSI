# -*- coding: utf-8 -*-
"""
Created on Sun Jul 25 20:36:46 2021

@author: study
"""

import numpy as np
import cv2

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

images=get_images('./pictureDataset/orange/')

images=[cv2.imread(i) for i in images]


lower_blue = np.array([0, 0, 0])
upper_blue = np.array([70, 255, 255])

lower_orange = np.array([100, 150, 100])
upper_orange = np.array([110, 255, 255])

lower_green = np.array([80, 50, 120])
upper_green = np.array([90, 255, 255])

kernel = np.ones((3,3),np.uint8)

for i in range(5):
    hsv = cv2.cvtColor(images[i], cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    res = cv2.bitwise_and(images[i], images[i], mask=mask)
    cv2.imshow('pic',res)
    cv2.waitKey(3000)



