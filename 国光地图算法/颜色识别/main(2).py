# coding=UTF-8
import cv2
import numpy as np
import serial
import RPi.GPIO as GPIO
import time
def perspective_transform(image, src_points, dst_points):
    # 计算透视变换矩阵
    perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # 进行透视变换
    transformed_image = cv2.warpPerspective(image, perspective_matrix, (image.shape[1], image.shape[0]))

    return transformed_image


def get_contours(u_image,color,color_name,big_times):
    u_image = cv2.cvtColor(u_image , cv2.COLOR_BGR2HSV)
    u_image = cv2.GaussianBlur(u_image, (5, 5), 0)  # 高斯滤波降噪，模糊图片
    kernel = np.ones((3, 3), np.uint8)  # 核定义
    if color_name=='red':
        color_img_1 = cv2.inRange(u_image, color[color_name]["color_lower"], color[color_name]["color_upper"])
        color_img_2 = cv2.inRange(u_image, color[color_name]["color2_lower"], color[color_name]["color2_upper"])
        color_img = cv2.addWeighted(color_img_1, 1, color_img_2, 1, 0)
    else:
        color_img = cv2.inRange(u_image, color[color_name]["color_lower"], color[color_name]["color_upper"])

    if color_name=='red' or color_name=='bule':
        color_img = cv2.dilate(color_img, kernel, iterations=big_times)  # 膨胀除去相关性小的颜色
    else :
        color_img = cv2.erode(color_img, kernel, iterations=2)  # 膨胀除去相关性小的颜色

    contours = cv2.findContours(color_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    return contours

def color_conflict(frame,color,color1_name,color2_name,big_times):
    win_color = 'None'
    u_image_1 = u_image_2 = frame.copy()
    contours_1 = get_contours(u_image_1, color, color1_name,big_times)
    contours_2 = get_contours(u_image_2, color, color2_name,big_times)
    contours_max = None
    if contours_1 or contours_2:
        if len(contours_2) == 0 and len(contours_1) == 0:
            pass
        elif len(contours_1) == 0 and len(contours_2):
            win_color = color2_name
            cnt_area_2 = [cv2.contourArea(x) for x in contours_2]  #提取轮廓面积信息
            cnt2_max_area = max(cnt_area_2)  # 寻找最大色块
            contours_max = contours_2[cnt_area_2.index(cnt2_max_area)]

        elif len(contours_2) == 0 and len(contours_1):
            win_color = color1_name
            cnt_area_1 = [cv2.contourArea(x) for x in contours_1]  #提取轮廓面积信息
            cnt1_max_area = max(cnt_area_1)  # 寻找最大色块
            contours_max = contours_1[cnt_area_1.index(cnt1_max_area)]
        else:
            cnt_area_1 = [cv2.contourArea(x) for x in contours_1]  #提取轮廓面积信息
            cnt1_max_area = max(cnt_area_1)  # 寻找最大色块
            contours_max_1 = contours_1[cnt_area_1.index(cnt1_max_area)]

            cnt_area_2 = [cv2.contourArea(x) for x in contours_2]  #提取轮廓面积信息
            cnt2_max_area = max(cnt_area_2)  # 寻找最大色块
            contours_max_2 = contours_2[cnt_area_2.index(cnt2_max_area)]
            if (cnt1_max_area > cnt2_max_area):
                win_color = color1_name
                contours_max = contours_max_1
            else:
                win_color = color2_name
                contours_max = contours_max_2
    return win_color,contours_max


if __name__ == '__main__':
    ####可调参数范围
    # 颜色RBG取值  color_lower 为HSV下限  color_upper 为HSV上限
    color = {
        "red": {"color_lower": np.array([156, 43, 46]), "color_upper": np.array([180, 255, 255]),
                "color2_lower": np.array([0, 43, 46]), "color2_upper": np.array([10, 255, 255])},
        'bule': {"color_lower": np.array([[100, 43, 46]]), "color_upper": np.array([124, 255, 255])},
        "green": {"color_lower": np.array([35, 43, 46]), "color_upper": np.array([77, 255, 255])},
        'yellow': {"color_lower": np.array([[26, 43, 46]]), "color_upper": np.array([34, 255, 255])},
    }
    cutting_factor = 0 #裁剪的部分比例  上半部分
    big_times = 4  #膨胀迭代次数
    cap = cv2.VideoCapture('/dev/video0') #摄像头选择  #头
    cap_2 = cv2.VideoCapture('/dev/video2')  # 摄像头选择  #尾巴
    img_shape = (640,640)  #图片放缩大小
    ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)   #115200为波特率
    filter_times = 5  #滤波次数
    ####可调参数范围
    cap_close = 0
    open_cap_2 = 0  #是否需要第二个摄像头  不用改，会自动找是否存在第二个摄像头
    if ser.isOpen():
        print("serial is open ")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(19,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)  #信号口  GPIO.PUD_UP
    GPIO.setup(26, GPIO.OUT)  # 头选择 high 为尾巴   一开始为输出

    color_result = np.zeros((filter_times,), dtype=np.int)
    if not cap.isOpened():
        print("capture error ")
        exit()
    else:
        print("capture is open")
    if not cap_2.isOpened():
        print("not capture2")
    else:
        open_cap_2 = 1
        print("capture2 is open")

    while(not GPIO.input(19)):  #等待小车应答
        GPIO.output(26, GPIO.HIGH)  #输出高电告诉小车 OK
    GPIO.output(26, GPIO.LOW)
    GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # 头选择 high 为尾巴

    while True:
        if open_cap_2 and GPIO.input(26):
            if cap_close==0:
                cap.release()
                cap_2 = cv2.VideoCapture('/dev/video2')
                color_result[...] = 0
            cap_close = 1
            ret ,frame = cap_2.read()
        else:
            if cap_close == 1:
                cap_2.release()
                cap = cv2.VideoCapture('/dev/video0')
                color_result[...] = 0
            cap_close = 0
            ret, frame = cap.read()

        if GPIO.input(19):
            if ret:
                frame = frame[int(frame.shape[0]*cutting_factor):,:]
                card_kind = 0#'None'
                frame =  cv2.resize(frame,img_shape)
                inside_img = frame.copy()
                ouside_color,out_side_contours_max =  color_conflict(frame,color,'bule','red',big_times)
                dst = np.float32([[0, 0], [frame.shape[1], 0], [0, frame.shape[0]], [frame.shape[1], frame.shape[0]]])
                src = np.float32([[0, 0], [0, 0], [0, 0], [0, 0]])
                if ouside_color:
                    x, y, w, h = cv2.boundingRect(out_side_contours_max)
                    src = np.float32([[x, y], [x + w, y], [x, y + h], [x + w, y + h]])
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    transformed_inside_image = perspective_transform(inside_img, src, dst)
                    inside_color, out_side_contours_max = color_conflict(transformed_inside_image, color, 'yellow', 'green',big_times)
                    if ouside_color == 'bule' and inside_color == 'yellow':
                        card_kind = 1#'out_bule,inside_yellow'
                    elif ouside_color == 'red' and inside_color == 'green':
                        card_kind = 2#'out_red,inside_green'
                    elif ouside_color == 'bule' and inside_color == 'green':
                        card_kind = 3#'out_bule,inside_green'
                    elif ouside_color == 'red' and inside_color == 'yellow':
                        card_kind = 4#'out_red,inside_yellow'
                    if(card_kind):
                        color_result = np.roll(color_result, 1)
                        color_result[0] = card_kind

                        flag  = 1
                        for color_data in color_result[1:]:
                            if color_data != color_result[0]:
                                flag = 0
                                break
                        if flag:
                            # print(color_result[0])
                            ser.write(bytes([0xff]) + bytes([color_result[0]])+bytes([0xaa]))
                        else:
                            ser.write(bytes([0xff]) + bytes([0])+bytes([0xaa]))
                    else:
                        ser.write(bytes([0xff]) + bytes([0]) + bytes([0xaa]))
                else:
                    ser.write(bytes([0xff]) + bytes([0]) + bytes([0xaa]))

                #cv2.imshow('TEST', frame)
                #print(card_kind,"aaa = ",cap_close)

        cv2.waitKey(1)
