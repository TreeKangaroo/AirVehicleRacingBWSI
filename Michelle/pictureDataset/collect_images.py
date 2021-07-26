# -*- coding: utf-8 -*-
"""
Created on Sun Jul 25 19:10:03 2021

@author: study
"""

import numpy as np
import cv2
from djitellopy import Tello

tello=Tello()

tello.connect()
tello.streamon()

for i in range (20):
    img=tello.get_frame_read().frame
    name='orange'+str(i)+'.jpg'
    cv2.imwrite(name,img)
    cv2.waitKey(1000)
    print('written')

tello.streamoff()
tello.end()