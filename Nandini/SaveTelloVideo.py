from djitellopy import Tello
import cv2 as cv
import time

tello = Tello()
tello.connect()
print("battery percentage {}".format(tello.get_battery()))
tello.streamon()

time.sleep(30)

frameReader = tello.get_frame_read()
frameWidth = frameReader.cap.get(cv.CAP_PROP_FRAME_WIDTH)
frameHt = frameReader.cap.get(cv.CAP_PROP_FRAME_HEIGHT)

print('frameWidth {}'.format(frameWidth))
print('frameHt {}'.format(frameHt))

streamWriter = cv.VideoWriter('samplevideo.avi', cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (int(frameWidth), int(frameHt)))

while True:
    img = frameReader.frame
    streamWriter.write(img)
    cv.imshow("SampleVideo", img)
    if cv.waitKey(3) & 0xff == ord('q'):
        break

streamWriter.release()
tello.streamoff()
cv.destroyAllWindows()

