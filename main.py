from picamera.array import PiRGBArray
from picamera import PiCamera
from e_drone.drone import *
from e_drone.protocol import *
from e_drone.system import *
import time
import datetime
from time import sleep
from cv2 import cvtColor , COLOR_BGR2HSV,threshold,THRESH_BINARY,THRESH_BINARY_INV,bitwise_and,flip,waitKey,imshow,destroyAllWindows, imread,inRange
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


    image = imread("/home/pi/Desktop/1.jpg")
    imshow("a", image)
    sleep(1)

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

                #강제종료

                if waitKey(1) & 0xff == ord('q'):
                    destroyAllWindows()
                    drone.sendStop()
                    drone.close()
                    print("WARNING")
                    exit(0)

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
                    if center_max_x ==0:
                        center_max_x = 639

                    for i in range(center_x1, min_x1, -1):
                        if bi_blue[center_y1][i] == 255 and i < center_min_x:
                            center_min_x = i
                            break
                    if center_min_x ==640:
                        center_min_x = 1

                    for j in range(center_y1, min_y1, -1):
                        if bi_blue[j][center_x1] == 255 and j < center_min_y:
                            center_min_y = j
                            break
                    if center_min_y ==480:
                        center_min_y = 1

                    for j in range(center_y1, max_y1):
                        if bi_blue[j][center_x1] == 255 and j > center_max_y:
                            center_max_y = j
                            break
                    if center_max_y ==0:
                        center_max_y = 479

                    center_x2 = int((center_min_x + center_max_x) / 2)
                    center_y2 = int((center_min_y + center_max_y) / 2)

                    rad_up = center_y2 - center_min_y
                    rad_down = center_max_y - center_y2
                    rad_left =  center_x2 - center_min_x
                    rad_right = center_max_x - center_x2

                    if rad_up > rad_down +30:
                        drone.sendControlPosition16(0, 0, 2, 5, 0, 0)
                        print("circle is on the top")
                        sleep(1)
                    elif rad_down > rad_up +30:
                        drone.sendControlPosition16(0, 0, -2, 5, 0, 0)
                        print("circle is under the drone")
                        sleep(1)

                    if rad_left > rad_right +30:
                        drone.sendControlPosition16(0, 2, 0, 5, 0, 0)
                        print("circle is on the left")
                        sleep(1)
                    elif rad_right > rad_left +30:
                        drone.sendControlPosition16(0, -2, 0, 5, 0, 0)
                        sleep(1)
                        print("circle is on the right")


                    #첫번째 링은 그냥 직진
                    if cnt == 0 and step == 0:
                        sleep(2)
                        drone.sendControlPosition16(18, 0, 0, 6, 0, 0)
                        sleep(5)
                        phase_1_1 = 0
                        phase_1_2 = 1
                        cnt = cnt + 1
                        step = 0
                        check = [0, 0]
                    #나머지 경우에는 중점인식
                    else:
                        if center_x2 < 305:  # 중점이 왼쪽에 있다. -> 왼쪽으로 가야한다.
                            drone.sendControlPosition16(0, 1, 0, 5, 0, 0)
                            sleep(3)
                            print("go to left")
                            print(center_x2, center_y2)
                        elif center_x2 > 335:  # 중점이 오른쪽에 있다. -> 오른쪽으로 가야한다.
                            drone.sendControlPosition16(0, -1, 0, 5, 0, 0)
                            sleep(3)
                            print("go to right")
                            print(center_x2, center_y2)
                        elif center_x2 >= 305 and center_x2 <= 335:
                            check[0] = 1

                        if center_y2 < 225:  # 중점이 아래에있다 - > 위로 가야한다.
                            drone.sendControlPosition16(0, 0, 1, 5, 0, 0)
                            sleep(3)
                            print("go to up")
                            print(center_x2, center_y2)
                        elif center_y2 > 255:  # 중점이 위에 있다. -> 아래로 가야한다.
                            drone.sendControlPosition16(0, 0, -1, 5, 0, 0)
                            sleep(3)
                            print("go to down")

                            print(center_x2, center_y2)
                        elif center_y2 >= 225 and center_y2 <= 255:
                            check[1] = 1

                        if check == [1, 1] and step == 0 and cnt == 0:
                            print("go to forward 18")
                            print(center_x2, center_y2)
                            drone.sendControlPosition16(18, 0, 0, 6, 0, 0)
                            sleep(5)
                            phase_1_1 = 0
                            phase_1_2 = 1
                            cnt = cnt + 1
                            check = [0, 0]
                        elif check == [1, 1] and step == 0 and cnt != 0 :
                            print("go to forward 25")
                            print(center_x2, center_y2)
                            drone.sendControlPosition16(25, 0, 0, 6, 0, 0)
                            sleep(5)
                            phase_1_1 = 0
                            phase_1_2 = 1
                            cnt = cnt + 1
                            check = [0, 0]

                        elif check == [1, 1] and step == 1:
                            print("go to forward 9 ")
                            print(center_x2, center_y2)
                            drone.sendControlPosition16(9, 0, 0, 5, 0, 0)
                            sleep(3)
                            phase_1_1 = 0
                            phase_1_2 = 1
                            cnt = cnt + 1
                            step = 0
                            check = [0, 0]



                if phase_1_2 == 1:
                    print(f"number of blue pixel is {np.sum(bi_blue) / 255} ")
                    if bi_blue[240][320] == 255:
                        sleep(2)
                        drone.sendControlPosition16(-5, 0, 0, 5, 0, 0)
                        sleep(2)
                        phase_1_1 = 1
                        phase_1_2 = 0
                        step = 1
                        cnt = cnt - 1

                    elif cnt != 3 :
                        bi_red = red_hsv(image)
                        value_th_red = np.where(bi_red[:, :] == 255)

                        min_x1_red = np.min(value_th_red[1])
                        max_x1_red = np.max(value_th_red[1])

                        if max_x1_red - min_x1_red < 25:
                            sleep(2)
                            drone.sendControlPosition16(1, 0, 0, 5, 0, 0)
                        else:
                            sleep(2)
                            drone.sendControlPosition16(0, 0, 0, 0, 90, 30)
                            sleep(4)
                            drone.sendControlPosition16(8, 0, 0, 5, 0, 0)
                            sleep(4)
                            drone.sendControlPosition16(0, 0, 1, 5, 0, 0)
                            sleep(4)
                            phase_1_1 = 1
                            phase_1_2 = 0
                            step = 0

                    elif cnt == 3:
                        print("Landing")
                        # 녹화 종료
                        drone.sendLanding()
                        sleep(5)
                        drone.close()
                        picam.stop_recording()
                        print(time.time() - start_time)
                        wc = False



    except Exception as e:
        print(e)
        drone.sendStop()
        sleep(2)
        picam.stop_recording()
        drone.close()


