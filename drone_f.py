from picamera.array import PiRGBArray
from picamera import PiCamera
from e_drone.drone import *
from e_drone.protocol import *
from e_drone.system import *
import time
import datetime
from cv2 import flip, inRange, bitwise_and, cvtColor, COLOR_BGR2HSV, threshold, THRESH_BINARY, THRESH_BINARY_INV, imwrite
import numpy as np

max_re_x = 640
max_re_y = 480

mx = 0
my = 0

def cam_setting(picam):
    picam.resolution = (max_re_x, max_re_y)
    picam.framerate = 32

def f_takeOff(drone):
    drone.sendTakeOff()
    print("TakeOff")
    sleep(5)

if __name__ == "__main__":
    picam = PiCamera()
    cam_setting(picam)
    rawCapture = PiRGBArray(picam, size=(max_re_x, max_re_y))
    drone = Drone()
    drone.open()
    cntr = 0
    

    
    try:
        now = datetime.datetime.now()
        f = now.strftime('%d %H:%M:%S')
        picam.start_recording(output=f + '.h264')
        f_takeOff(drone)
        


        drone.sendControlPosition16(3, 0, 0, 5, 0, 0)
        print("go f")
        sleep(7)
        drone.sendControlPosition16(0, 0, 0, 0, 90, 20)
        print("see left")
        sleep(2)
        cntr += 1



        #while cntr < 3:
        for frame in picam.capture_continuous(rawCapture, format='bgr', use_video_port=True):
            image = frame.array
            sleep(0.01)
            image = flip(image, 0)
            image = flip(image, 1)
            rawCapture.truncate(0)
            
            image_hsv = cvtColor(image, COLOR_BGR2HSV)
            H = image_hsv[:, :, 0]
            _, bi_H = threshold(H, 162, 255, THRESH_BINARY)
            _, bi_H_ = threshold(H, 192, 255, THRESH_BINARY_INV)
            image = bitwise_and(bi_H, bi_H_)
            imwrite("see.jpg",image)
            sleep(0.1)



            value_th = np.where(image[:, :] == 255)
            min_x1 = np.min(value_th[1])
            max_x1 = np.max(value_th[1])
            min_y1 = np.min(value_th[0])
            max_y1 = np.max(value_th[0])
            center_x1 = int((min_x1 + max_x1) / 2)
            center_y1 = int((min_y1 + max_y1) / 2)

            if center_x1 > 320:
                mx = center_x1 - 320
                mx *= -0.005
            else:
                mx = 320 - center_x1
                mx *= 0.005
            
            if center_y1 > 320:
                my = center_y1 - 320
                my *= 0.005
            else:
                my = 320 - center_y1
                my *= -0.005

            drone.sendControlPosition(3.5, mx, my, 0.5, 0, 0)
            sleep(8)

        drone.sendLanding()
        sleep(1)
        drone.close()


                



                




    except Exception as e:
        print(e)
        drone.sendLanding()
        sleep(1)
        drone.close()
        exit(0)
