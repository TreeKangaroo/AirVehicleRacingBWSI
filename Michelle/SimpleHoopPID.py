from warnings import resetwarnings
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
frame_generator=tello.get_frame_read()
frame=frame_generator.frame
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
yaw_controller = PID(0.3, -0.000000000000002, 0.1, MV_bar=5)
yaw_controller.send(None)

z_controller = PID(0.2, 0, 0.05)
z_controller.send(None)

x_controller = PID(0.07,-0.000000000005, 0.05)
x_controller.send(None)

y_controller = PID(0.5, 0, 0.03)
y_controller.send(None)

#Pool Noodle Thresholds
lower_green = np.array([70, 50, 70])
upper_green = np.array([95, 255, 255])

lower_blue = np.array([0, 30, 30])
upper_blue = np.array([50, 255, 255])

lower_orange = np.array([110, 20, 70])
upper_orange = np.array([125, 255, 230])

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
    
    last_yaw=0
    target_distance = 150
    kyawx=3
    
    state = 0
    count = 0
    target = 30
    targetOffset = 140
    font = cv.FONT_HERSHEY_SIMPLEX
    print("State 0: Line up with noodle")
    while inp != 'q':
        t = time.time()
        frame = frame_generator.frame
        frameCenter = (frame.shape[1] // 2, frame.shape[0] //2 - targetOffset)
        res = frame

        if count%3 == 0:
            hoop = Hoop(frame, lower_blue, upper_blue)
        elif count%3 == 1:
            hoop = Hoop(frame, lower_green, upper_green)
        else:
            hoop = Hoop(frame, lower_orange, upper_orange)

        if state == 0: #Line up with noodle
            y = 20
            if hoop.seenHoop:

                res = hoop.res
                c = hoop.contour
                cv.ellipse(res, hoop.ellipse, (0, 255, 0), 5)
                rectX,rectY,rectW,rectH = hoop.rect
                cv.drawContours(res, hoop.contours, -1, (255, 0, 0), 2) #Green fitted ellipse
                cv.rectangle(res,(rectX,rectY),(rectX+rectW,rectY+rectH),(0,255,0),5) #Green bounding rectangle
                cv.circle(res, hoop.center, 5, (255, 0, 0), -1) #Red circle in center of hoop
                
                cv.line(res, hoop.center, tuple(hoop.imgpts[0,0]), (255,0,0),5)
                cv.line(res, hoop.center, tuple(hoop.imgpts[1,0]), (0,255,0), 5)
                cv.line(res, hoop.center, tuple(hoop.imgpts[2,0]),(0,0,255), 5)
                
                    
                pitch_ratio=abs(math.tan(hoop.euler[0]))
                yaw_angle=abs(int(hoop.euler[1]))
                #yaw_comp_x=kyawx*math.sin(yaw_angle)
                distance=(np.sum(hoop.tvecs**2))**0.5
                
                #z_pitchcomp=y*pitch_ratio
                if yaw_angle>last_yaw:
                    yaw_angle=-1*yaw_angle
                    last_yaw=abs(yaw_angle)
                    
                z = int(z_controller.send((t, hoop.center[1], frameCenter[1])))
                x = -1 * int(x_controller.send((t, hoop.center[0], frameCenter[0])))
                y = -1*int(y_controller.send((t, distance, target_distance)))
                yaw=int(yaw_controller.send((t, yaw_angle, 0)))
                
                if (rectY < 80): textOrigin = (rectX, rectY + rectH + 40)
                else: textOrigin = (rectX, rectY-10)
                hoopText = 'Pitch:{:4d} Yaw:{:3d} Dist:{}'.format(int(hoop.euler[0]), yaw_angle, distance)
                cv.putText(res, hoopText, textOrigin, font, 0.75, (255, 255, 255), 2, cv.LINE_AA)

                bottomText = 'State:{} X:{:3d} Y:{:3d} Z:{:3d} Yaw: {:3d} PitchRatio:{:.3f}'.format(state, x, y, z, yaw, pitch_ratio)
                print(bottomText)
                cv.putText(res, bottomText, (0,40), font, 1, (255, 255, 255), 2, cv.LINE_AA)

                z_error = frameCenter[1] - hoop.center[1]
                x_error = frameCenter[0] - hoop.center[0]
                distance_error = abs(distance - target_distance)
                error_mag = math.sqrt(z_error**2 + x_error**2)
                #print(z_error, x_error, error_mag)
                if (error_mag < target and x < 20 and z < 20 and yaw<15 and distance_error<20): 
                    state = 1 #Comment out this line to debug with a single hoop
                    print("State 1: Approach noodle")
        elif state == 1:
            y = 50
            if hoop.seenHoop: #Don't correct position anymore because ellipse becomes wonky closeup
                yaw_angle=abs(int(hoop.euler[1]))
                
                z = int(z_controller.send((t, hoop.center[1], frameCenter[1])))
                x = -1 * int(x_controller.send((t, hoop.center[0], frameCenter[0])))

                if abs(z) > 20: z = 0
                if abs(x) > 20: x = 0

            else:
                state = 2
                lastSeen = time.time()
                print("State 2: Fly through noodle")
        elif state == 2:
            yaw = 0
            x = 0
            z = 0
            y = 50
            if time.time() - lastSeen >= 0.2:
                state = 3
                print("State 3: Next Hoop")
        else:
            count += 1
            state = 0

        if not debug: tello.send_rc_control(x, y, z, yaw)

        cv.line(res, frameCenter, (frameCenter[0]+x, frameCenter[1]-z), (0, 0, 255), 2)#Movement vector representation
        cv.circle(res, frameCenter, target, (0, 0, 255), 2) #Red circle of target radius
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