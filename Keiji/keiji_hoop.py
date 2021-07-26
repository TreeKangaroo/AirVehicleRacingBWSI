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
debug = False
if not debug: 
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

z_controller = PID(0.1, 0, 0.02)
z_controller.send(None)

x_controller = PID(100, 0, 0)
x_controller.send(None)

y_controller = PID(0.1, 0, 0)
y_controller.send(None)

#Pool Noodle Thresholds
lower_blue = np.array([0, 0, 0])
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


yaw = 0
z = 0
y = 0
x = 0
def main():
    global inp
    while inp != 'q':
        frame = tello.get_frame_read().frame
        hsv = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        mask = cv.erode(mask, kernel, iterations=1)
        res = cv.bitwise_and(frame, frame, mask=mask)

        frameCenter = (frame.shape[1] // 2, frame.shape[0] //2)
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key = cv.contourArea)
            x,y,w,h = cv.boundingRect(c)
            ellipse = cv.fitEllipse(c)
            hoopCenter = (int(ellipse[0][0]), int(ellipse[0][1]))

            cv.drawContours(res, contours, -1, (255, 0, 0), 2)
            cv.rectangle(res,(x,y),(x+w,y+h),(0,255,0),5)
            cv.circle(res, hoopCenter, 5, (255, 0, 0), -1)
            cv.ellipse(res, ellipse, (0, 255, 0), 5)



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