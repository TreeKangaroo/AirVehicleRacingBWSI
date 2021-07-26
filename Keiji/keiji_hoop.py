from djitellopy import Tello
import cv2 as cv
import numpy as np
import queue
import threading 

q = queue.Queue()
tello = Tello()
tello.connect()
print(tello.get_battery())

tello.streamon()

inp = '0'

lower_blue = np.array([0, 0, 0])
upper_blue = np.array([70, 255, 255])

lower_orange = np.array([100, 150, 100])
upper_orange = np.array([110, 255, 255])

lower_green = np.array([80, 50, 120])
upper_green = np.array([90, 255, 255])

kernel = np.ones((20,20),np.uint8)

def get_input():
    global inp
    while True:
        inp = input()
        print(inp)

def main():
    global inp
    while inp != 'q':
        frame = tello.get_frame_read().frame
        hsv = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        res = cv.bitwise_and(frame, frame, mask=mask)

        cv.imshow("camera", res)
        key = cv.waitKey(5)
        if key & 0xFF == ord('q'):
            print("DONE")
            tello.end()
            break
    tello.end()

if __name__=='__main__':
    t1 = threading.Thread(target=main)
    t2 = threading.Thread(target=get_input)

    t1.start()
    t2.start()