from picamera.array import PiRGBArray
from picamera import PiCamera
from e_drone.drone import *
from e_drone.protocol import *
from e_drone.system import *
import time
import datetime
from time import sleep
from cv2 import cvtColor , COLOR_BGR2HSV,threshold,THRESH_BINARY,THRESH_BINARY_INV,bitwise_and,flip,waitKey,imshow,destroyAllWindows, imread,inRange,imwrite
import numpy as np


# 카메라 세팅
def cam_setting(picam):
    # picam 메뉴얼 URL : https://picamera.readthedocs.io/en/release-1.10/api_camera.html
    # 받아오는 카메라 해상도 설정
    picam.resolution = (640, 480)
    # 카메라의 프레임 설정
    picam.framerate = 32


# 드론 이륙
def f_takeOff(drone):
    drone.sendTakeOff()
    print("TakeOff")
    sleep(5)


# 빨간색 hsv로 변환
def red_hsv(image):
    image_hsv = cvtColor(image, COLOR_BGR2HSV)
    H = image_hsv[:, :, 0]
    _, bi_H = threshold(H, 172, 255, THRESH_BINARY)
    _, bi_H_ = threshold(H, 182, 255, THRESH_BINARY_INV)

    bi_H_r = bitwise_and(bi_H, bi_H_)
    return bi_H_r


def blue_hsv(image):
    image_hsv = cvtColor(image, COLOR_BGR2HSV)
    '''H = image_hsv[:, :, 0]
    _, bi_H = threshold(H, 95, 255, THRESH_BINARY)
    _, bi_H_ = threshold(H, 105, 255,THRESH_BINARY_INV)

    bi_H_r = bitwise_and(bi_H, bi_H_)
    return bi_H_r'''
    th_low = (90, 80, 70)
    th_high = (120, 255, 255)

    img_th = inRange(image_hsv, th_low, th_high)
    return img_th

def puple_hsv(image):
    image_hsv = cvtColor(image, COLOR_BGR2HSV)
    '''H = image_hsv[:, :, 0]
    _, bi_H = threshold(H, 95, 255, THRESH_BINARY)
    _, bi_H_ = threshold(H, 105, 255,THRESH_BINARY_INV)

    bi_H_r = bitwise_and(bi_H, bi_H_)
    return bi_H_r'''
    th_low = (50, 10, 50)
    th_high = (200, 200, 255)

    img_th = inRange(image_hsv, th_low, th_high)
    return img_th


if __name__ == "__main__":  # 이 파일을 직접 실행했을 경우 __name__ = "__main__"이 됨
    # 파이캠 설정
    picam = PiCamera()
    cam_setting(picam)
    rawCapture = PiRGBArray(picam, size=(640, 480))
    # drone 인스턴스 선언
    drone = Drone()
    # drone 인스턴스 시작
    drone.open()
    # 변수 설정
    # ---------------------------------
    phase_1_1 = 1
    phase_1_2 = 0
    step = 0
    check = [0, 0]
    back = 0
    wc = True
    cnt = 0
    find_num = 0

    start_time = time.time()
    now = datetime.datetime.now()
    f = now.strftime('%d %H:%M:%S')
    picam.start_recording(output=f + '.h264')  # 녹화 시작
    # 이륙
    f_takeOff(drone)

    try:
        while (wc):

            for frame in picam.capture_continuous(rawCapture, format='bgr', \
                                                  use_video_port=True):



                # image 변수에 frame의 배열 저장 - Numpy 형식
                image = frame.array
                sleep(0.01)



                # 영상 x, y축 반전
                image = flip(image, 0)
                image = flip(image, 1)


                rawCapture.truncate(0)

                # 첫번째 링일 때
                if phase_1_1 == 1:
                    bi_blue = blue_hsv(image)
                    value_th = np.where(bi_blue[:, :] == 255)
                    drone.sendControlPosition16(15, 0, 0, 5, 0, 0)
                    sleep(5)
                    drone.sendLanding()
                    sleep(2)
                    drone.close()






    except Exception as e:
        print(e)
        drone.sendStop()
        sleep(2)
        picam.stop_recording()
        drone.close()


