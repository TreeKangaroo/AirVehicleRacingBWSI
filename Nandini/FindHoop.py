from djitellopy import Tello
import cv2 as cv
import numpy as np
import threading
import time
import sys
import math
sys.path.append('../')
# from General.Hoop import Hoop

class Hoop:

    def __init__(self, img, lower_thresh, upper_thresh):
        import cv2
        # import numpy as np
        # import math

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        self.mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
        kernel = np.ones((20, 20), np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        self.res = cv2.bitwise_and(img, img, mask=self.mask)

        self.contours, hierarchy = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.seenHoop = False
        if len(self.contours) > 0:
            self.contour = max(self.contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(self.contour)
            self.rect = (x, y, w, h)
            if self.contour.shape[0] > 5:
                self.ellipse = cv2.fitEllipse(self.contour)
                self.area = int(self.ellipse[1][0] * self.ellipse[1][1] * math.pi)
                if self.area > 150:  # original 500
                    self.seenHoop = True
                self.center = (int(self.ellipse[0][0]), int(self.ellipse[0][1]))
            else:
                self.center = (int(x + w // 2), int(y + h // 2))

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
        print(f'MV:{int(MV)} P:{int(P)} I:{int(I)} D:{int(D)} e:{e} e_prev:{e_prev}')

        e_prev = e
        t_prev = t

#PID Values
yaw_controller = PID(0.2, 0, 0.02)
yaw_controller.send(None)

z_controller = PID(0.2, 0, 0.05)
z_controller.send(None)

x_controller = PID(0.12, 0, 0.03)
x_controller.send(None)

y_controller = PID(0.1, 0, 0)
y_controller.send(None)

#Pool Noodle Thresholds
lower_blue = np.array([0, 100, 0])
upper_blue = np.array([70, 255, 255])

# lower_orange = np.array([90, 150, 100])
# upper_orange = np.array([180, 255, 255])

lower_orange = np.array([255, 91, 89])
upper_orange = np.array([255, 91, 89])

lower_green = np.array([70, 50, 120])
upper_green = np.array([90, 255, 255])

# kernel = np.ones((20,20),np.uint8)

# Call this function after the drone takesoff
# this function assumes streamon is done
# pass "blue", "green" or "orange" for hoopColor argument
# returns tuple drone height and found flag
def FindHoop(drone: Tello, height: int, hoopColor: str):
    print('FindHoop(): function')
    initialHeight = drone.get_height()
    print(f'initial height: {initialHeight}')
    print(f'height received: {height}')
    droneHeight = initialHeight

    if abs(height - initialHeight) < 20:
        print("cannot move up/down with height adjustment less than 20")
        return droneHeight, False
    if initialHeight < height:
        drone.move_up(height - initialHeight)
    elif initialHeight > height:
        drone.move_down(initialHeight - height)

    frameReader = drone.get_frame_read()
    currentYaw = drone.get_yaw()
    yawOn = False
    while True:
        imgObj = frameReader.frame
        if hoopColor == "blue":
            thisHoop = Hoop(imgObj, lower_blue, upper_blue)
        elif hoopColor == "green":
            thisHoop = Hoop(imgObj, lower_green, upper_green)
        elif hoopColor == "orange":
            thisHoop = Hoop(imgObj, lower_orange, upper_orange)
        else:
            thisHoop = None

        if thisHoop != None and thisHoop.seenHoop:
            print('found hoop')
            drone.send_rc_control(0, 0, 0, 0)
            droneHeight = drone.get_height()
            found = True
            break
        else:
            print('not found yet')
            if not yawOn:
                # print('Send yaw velocity 50')
                # drone.send_rc_control(0, 0, 0, 50)
                yawOn = True
            newYaw = drone.get_yaw()
            if newYaw == currentYaw:
                print('full rotation reached')
                droneHeight = drone.get_height()
                found = False
                break

    return droneHeight, found

def get_input():
    global inp
    while True:
        inp = input()
        print(inp)

def main():
    global inp
    startTime = time.time()
    timeElapsed = 0
    adjHt = tello.get_height()
    tello.send_rc_control(0, 0, 0, 50) # set it spin
    while inp != 'q':
        frame = tello.get_frame_read().frame
        cv.imshow("camera", frame)
        key = cv.waitKey(1)
        if key & 0xFF == ord('q'):
            print("q pressed")
            break
        if (timeElapsed > 60):
            print('timeout')
            break
        adjHt = adjHt - 20
        if adjHt < 20:
            print('lower ht limit breached')
            break
        telloHt, foundHoop = FindHoop(tello, adjHt, "green")
        print(f'hoop fouund?: {foundHoop} at height {telloHt}')
        if foundHoop:
            print('Found!!')
            tello.send_rc_control(0, 0, 0, 0)
            break
        curTime = time.time()
        timeElapsed = curTime - startTime
    print('ENDING')
    cv.destroyAllWindows()
    tello.land()
    tello.streamoff()
    tello.end()
    exit()

tello = Tello()
tello.connect()
print(f'Batter charge left: {tello.get_battery()}')
tello.streamon()
inp = '0'
# tello.send_rc_control(0,0,0,0)
tello.takeoff()
print(f'Takeoff height: {tello.get_height()}')
tello.move_up(50)

if __name__=='__main__':
    t1 = threading.Thread(target=main)
    t2 = threading.Thread(target=get_input)
    print('starting key input thread')
    t1.start()
    print('starting main thread')
    t2.start()
