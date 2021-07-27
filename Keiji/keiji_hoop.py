from djitellopy import Tello
import cv2 as cv
import numpy as np
import threading
import time
import sys
import math
sys.path.append('../')
from General.Hoop import Hoop

tello = Tello()
tello.connect()
print(tello.get_battery())
tello.streamon()
inp = '0'
debug = False
if not debug: 
    tello.send_rc_control(0,0,0,0)
    tello.takeoff()

def PID(Kp, Ki, Kd, MV_bar=0):
    e_prev = 0
    t_prev = -1
    I = 0
    MV = MV_bar
    while True:
        t, PV, SP = yield MV
        e = SP - PV
        P = Kp * e
        I = I + Ki*e*(t - t_prev)
        D = Kd*(e - e_prev)/(t - t_prev)
        MV = MV_bar + P + I + D
        #print(f'MV:{int(MV)} P:{int(P)} I:{int(I)} D:{int(D)} e:{e} e_prev:{e_prev}')

        e_prev = e
        t_prev = t

#PID Values
yaw_controller = PID(0.2, 0, 0.02)
yaw_controller.send(None)

z_controller = PID(0.1, 0, 0.05)
z_controller.send(None)

x_controller = PID(0.1, 0, 0.01)
x_controller.send(None)

y_controller = PID(0.1, 0, 0)
y_controller.send(None)

#Pool Noodle Thresholds
lower_blue = np.array([10, 120, 0])
upper_blue = np.array([30, 255, 255])

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
    yaw = 0
    z = 0
    y = 0
    x = 0
    state = 0
    print("State 0: Line up with noodle")
    while inp != 'q':
        t = time.time()
        frame = tello.get_frame_read().frame
        frameCenter = (frame.shape[1] // 2, frame.shape[0] //2)
        res = frame
        hoop = Hoop(frame, lower_blue, upper_blue)
        if state == 0: #Line up with noodle
            y = 0
            if hoop.seenHoop:
                res = hoop.res
                c = hoop.contour
                cv.ellipse(res, hoop.ellipse, (0, 255, 0), 5)
                rectX,rectY,rectW,rectH = hoop.rect
                cv.drawContours(res, hoop.contours, -1, (255, 0, 0), 2) #Green fitted ellipse
                cv.rectangle(res,(rectX,rectY),(rectX+rectW,rectY+rectH),(0,255,0),5) #Green bounding rectangle
                cv.circle(res, hoop.center, 5, (255, 0, 0), -1) #Red circle in center of hoop

                z = int(z_controller.send((t, hoop.center[1], frameCenter[1] - 120)))
                x = -1 * int(x_controller.send((t, hoop.center[0], frameCenter[0])))

                z_error = abs(frameCenter[1] - 120 - hoop.center[1])
                x_error = abs(frameCenter[0] - hoop.center[0])

                if (z_error < 70 and x_error < 70): 
                    state = 1
                    print("State 1: Approach noodle")
        elif state == 1:
            y = 50
            if hoop.seenHoop:
                z = 0 #int(z_controller.send((t, hoop.center[1], frameCenter[1] - 120)))
                x = 0 #-1 * int(x_controller.send((t, hoop.center[0], frameCenter[0])))
            else:
                state = 2
                lastSeen = time.time()
                print("State 2: Fly through noodle")
        elif state == 2:
            yaw = 0
            x = 0
            z = 0
            y = 50
            if time.time() - lastSeen >= 1:
                state = 3
                print("State 3: Land")
        else:
            break

        if not debug: tello.send_rc_control(x, y, z, yaw)

        cv.circle(res, frameCenter, 5, (0, 0, 255), -1) #Blue circle in center of screen
        cv.imshow("camera", res)
        key = cv.waitKey(1)
        if key & 0xFF == ord('q'):
            print("DONE")
            tello.end()
            break
    print('ENDING')
    tello.send_rc_control(0,0,0,0)
    tello.end()
    exit()

if __name__=='__main__':
    t1 = threading.Thread(target=main)
    t2 = threading.Thread(target=get_input)
    t1.start()
    t2.start()