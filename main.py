from picamera.array import PiRGBArray
from picamera import PiCamera
from e_drone.drone import *
from e_drone.protocol import *
from e_drone.system import *
import time
from time import sleep
import cv2
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
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H = image_hsv[:, :, 0]
    _, bi_H = cv2.threshold(H, 172, 255, cv2.THRESH_BINARY)
    _, bi_H_ = cv2.threshold(H, 182, 255, cv2.THRESH_BINARY_INV)

    bi_H_r = cv2.bitwise_and(bi_H, bi_H_)
    return bi_H_r


def blue_hsv(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H = image_hsv[:, :, 0]
    _, bi_H = cv2.threshold(H, 95, 255, cv2.THRESH_BINARY)
    _, bi_H_ = cv2.threshold(H, 105, 255, cv2.THRESH_BINARY_INV)

    bi_H_r = cv2.bitwise_and(bi_H, bi_H_)
    return bi_H_r


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
    phase_1 = 1
    step = 0
    # ---------------------------------
    phase_2 = 0
    ud_flag = 0
    first_sight = False
    # ---------------------------------
    phase_3 = 0
    PHASE3_PIXEL = 20000
    lr_detection = False
    lr_flag = 0
    i=1
    # 이륙

    start_time = time.time()
    while (True):
        try:
            for frame in picam.capture_continuous(rawCapture, format='bgr', \
                                                  use_video_port=True):
                # image 변수에 frame의 배열 저장 - Numpy 형식
                image = frame.array
                sleep(0.01)
                # 영상 x, y축 반전
                image = cv2.flip(image, 0)
                image = cv2.flip(image, 1)

                test_blue = blue_hsv(image)

                value_th = np.where(test_blue[:, :] == 255)

                cv2.imshow("Frame", test_blue)
                key = cv2.waitKey(0) & 0xFF

                rawCapture.truncate(0)

                if key == ord("c"):
                    continue
                elif key == ord("e"):
                    cv2.imwrite("capture_{}.jpg".format(i), image)
                elif key == ord("q"):
                    cv2.destroyAllWindows()
                    break







        except Exception as e:
            print(e)
            # drone.sendLanding()
            drone.close()