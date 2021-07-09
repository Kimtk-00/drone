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
    phase_1_1 = 1
    phase_1_2 = 0
    step = 0
    check = [0, 0]
    back = 0
    # ---------------------------------
    phase_2 = 0
    ud_flag = 0
    first_sight = False
    # ---------------------------------
    phase_3 = 0
    PHASE3_PIXEL = 20000
    lr_detection = False
    lr_flag = 0
    # 이륙
    f_takeOff(drone)

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

                rawCapture.truncate(0)
                # 첫번째 링일 때
                if phase_1_1 == 1:
                    bi_blue = blue_hsv(image)

                    value_th = np.where(bi_blue[:, :] == 255)

                    min_x1 = np.min(value_th[1])
                    max_x1 = np.max(value_th[1])
                    min_y1 = np.min(value_th[0])
                    max_y1 = np.max(value_th[0])

                    center_x1 = int((min_x1 + max_x1) / 2)
                    center_y1 = int((min_y1 + max_y1) / 2)

                    center_min_x = 640
                    center_max_x = 0
                    center_min_y = 480
                    center_max_y = 0

                    for i in range(center_x1, max_x1-5):
                        if bi_blue[center_y1][i] == 255 and i > center_max_x:
                            center_max_x = i
                            break

                    for i in range(center_x1, min_x1, -1):
                        if bi_blue[center_y1][i] == 255 and i < center_min_x:
                            center_min_x = i
                            break

                    for j in range(center_y1, min_y1, -1):
                        if bi_blue[j][center_x1] == 255 and j < center_min_y:
                            center_min_y = j
                            break

                    for j in range(center_y1, max_y1):
                        if bi_blue[j][center_x1] == 255 and j > center_max_y:
                            center_max_y = j
                            break

                    center_x2 = int((center_min_x + center_max_x) / 2)
                    center_y2 = int((center_min_y + center_max_y) / 2)

                    if center_x2 > 640:
                        center_x2 = 640
                    if center_x2 < 0:
                        center_x2 = 0

                    if center_y2 > 480:
                        center_y2 = 480
                    if center_y2 < 0:
                        center_y2 = 0

                    if center_x2 < 310:  # 중점이 왼쪽에 있다. -> 왼쪽으로 가야한다.
                        #drone.sendControlPosition16(0, 1, 0, 1, 0, 0)
                        sleep(1)
                        print("go to left")
                        print(center_x2, center_y2)
                    elif center_x2 > 330:  # 중점이 오른쪽에 있다. -> 오른쪽으로 가야한다.
                        #drone.sendControlPosition16(0, -1, 0, 1, 0, 0)
                        sleep(1)
                        print("go to right")
                        print(center_x2, center_y2)
                    elif center_x2 >= 310 and center_x2 <= 330:
                        check[0] = 1

                    if center_y2 < 230:  # 중점이 아래에있다 - > 위로 가야한다.
                        #drone.sendControlPosition16(0, 0, 1, 1, 0, 0)
                        sleep(1)
                        print("go to up")
                        print(center_x2, center_y2)
                    elif center_y2 > 250:  # 중점이 위에 있다. -> 아래로 가야한다.
                        #drone.sendControlPosition16(0, 0, -1, 1, 0, 0)
                        sleep(1)
                        print("go to down")
                        print(center_x2, center_y2)
                    elif center_y2 >= 230 and center_y2 <= 250:
                        check[1] = 1

                    if check == [1, 1]:
                        print("go to forward")
                        print(center_x2,center_y2)
                        drone.sendControlPosition16(15, 0, 0, 5, 0, 0)
                        sleep(5)
                        phase_1_2 = 1
                        phase_1_1 = 0

                        print("Landing")
                        #drone.sendLanding()
                        drone.close()




        except Exception as e:
            print(e)
            # drone.sendLanding()
            drone.close()