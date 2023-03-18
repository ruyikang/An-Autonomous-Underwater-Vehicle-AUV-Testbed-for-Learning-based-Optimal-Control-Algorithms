import cv2
import numpy as np
import datetime
import math
import keyboard
import time
import cvzone

import joblib
model = joblib.load(r"DTC_task1.pkl")

import serial
arduinoData = serial.Serial('com3', 115200)

def main():

    # configuration
    last_distance = [48,48,48] # blue, green, black
    last_yaw = [0,0,0] # blue, green, black
    pref_long = 0  #long pixel for contour
    pref_short = 0  #short pixel for contour
    plate_type = ["blue", "green", "black"]
    res_index = 0
    counter = 0
    switch = 0
    distance = 48
    dataline_blue =(0,0)
    dataline_green = (0, 0)
    dataline_black = (0, 0)

    topwater = 48 # 顶部（潜艇付出水面） #actually 48    # 水深23cm 从48-72cm 但submarine高5cm，所以深度为67cm
    #cap = cv2.VideoCapture(1) # for macbook
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)   # open 1080hp camera (0), computer camera is (1)
    fps = 60
    cap.set(6, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # MJPG
    cap.set(5, fps)  # fps

    #1280x720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    isOpened = cap.isOpened()   # judge is it opened or not
    yaw_actual = 0
    yaw_actual1 = 0

    # color definition
    color_dict = dict()
    lower_blue_hsv = np.array([100, 43, 46])
    upper_blue_hsv = np.array([124, 255, 255])
    #lower_red_hsv = np.array([156, 43, 46])
    #upper_red_hsv = np.array([180, 255, 255])
    lower_green_hsv = np.array([35, 43, 46])
    upper_green_hsv = np.array([77, 255, 255])
    lower_black_hsv = np.array([0, 0, 46])
    upper_black_hsv = np.array([180, 43, 150])
    color_dict['blue'] = [lower_blue_hsv, upper_blue_hsv]
    color_dict['green'] = [lower_green_hsv, upper_green_hsv]
    color_dict['black'] = [lower_black_hsv, upper_black_hsv]

    #template = cv2.imread("template.png",0)
# width and centimeters
    f = 0.0028 # 焦距2.8mm
    pi = 0.000003 # pixel cm
    W = 2.6 #cm real value for submarine
    Wb = 2.5 # cm for black one
    # d =  #cm distance real value
    # w =  # pixel value
    # f = (w*d)/W
    # d = (f*W)/w

# function definition
    width = [50,47,46.68,44.8,40]
    height_blue = [200,187.421,182.789,173.5263,156] # pref_long
    height_black = [149,137.421,132.789,123.5263,105] # length
    #width1 = [50,41]
    x_pixel_sample = [110,171,195,243.2,340]
    y_pixel_sample = [20,62,79,113,180]
    conver_x_sample = [15.27778,15.88889,16.17857,13.57193,9.971831] # rate for conversion between pixel and real length
    conver_y_sample = [16.77273,14.31579,13.22727,12.09091,8.47619]
    depth = [48,53,55,59,67] # y=Ax+B  OR y = Ax^2+bx+C
    #depth1 = [48,67]
    coff_xpixel = np.polyfit(depth,x_pixel_sample,2)
    equ1 = np.poly1d(coff_xpixel)
    coff_ypixel = np.polyfit(depth,y_pixel_sample,2)
    equ2 = np.poly1d(coff_ypixel)
    coff_xconver = np.polyfit(depth,conver_x_sample,2)
    equ3 = np.poly1d(coff_xconver)
    coff_yconver = np.polyfit(depth,conver_y_sample,2)
    equ4 = np.poly1d(coff_yconver)
    coff_width = np.polyfit(width, depth, 2)
    equ5 = np.poly1d(coff_width)
    coff_heightblue = np.polyfit(height_blue, depth, 2)
    equ6 = np.poly1d(coff_heightblue)
    coff_heightblack = np.polyfit(height_black, depth, 2)
    equ7 = np.poly1d(coff_heightblack)

    while isOpened:
        counter = counter + 1
        flag, img = cap.read()  # read every frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray1 = gray
        gray1[gray1 > 200] = 255
        gray1[gray1 < 50] = 0  # 把中间像素转化为黑与白
        blur = cv2.medianBlur(gray1, 3)  # median blur
        ret, dst = cv2.threshold(blur, 105, 255, cv2.THRESH_BINARY_INV) # 与环境有关, can be adjusted 前值越小 越能放噪音 但太小会导致本身轮廓消失
        kernel = np.ones((3, 3), np.uint8)
        obj_bin = cv2.erode(dst, kernel, iterations=1)
        kernel = np.ones((3, 3), np.uint8)
        obj_bin = cv2.dilate(obj_bin, kernel, iterations=1)
        contours, hier = cv2.findContours(obj_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        AREA_THRESHOLD_L = 4000  #4000
        AREA_THRESHOLD_H = 30000  #20000    define the contour area
        canvas = np.copy(img)
        # top 4 points
        cv2.circle(canvas, (110, 20), 3, (255, 0, 0), -1)  # boundry left-top point # top from 6 cm
        cv2.circle(canvas, (110, 700), 3, (255, 0, 0), -1)  # boundry left-bottom point
        cv2.circle(canvas, (1170, 20), 3, (255, 0, 0), -1)  # boundry right-top point
        cv2.circle(canvas, (1170, 700), 3, (255, 0, 0), -1)  # boundry right-bottom point

        # bottom 4 points
        cv2.circle(canvas, (340, 180), 3, (255, 0, 255), -1)  # boundry left-top point # bottom
        cv2.circle(canvas, (340, 540), 3, (255, 0, 255), -1)  # boundry left-bottom point
        cv2.circle(canvas, (940, 180), 3, (255, 0, 255), -1)  # boundry right-top point
        cv2.circle(canvas, (940, 540), 3, (255, 0, 255), -1)  # boundry right-bottom point
        # 940-340 = 600 = 62.6cm   9.585=1cm   experimental  9.571=1cm  540-180 = 360 = 40.4cm  8.911 = 1cm experimental 10.29 = 1cm average 9.088=1cm d=62cm bottom
        # 1170-110 = 1020 = 62.6cm   16.294 = 1cm     700-20 = 680 = 40.4cm   16.832 = 1cm  average 16.563=1cm d=36cm top

        #48cm
        #上下点距离差距31cm
        #摄像头离最底下潜水艇67cm
        #d和w之间是反比关系， 因为焦距和物体真实大小都是固定的

        obj_rect_cnt_list = []
        for i, cnt in enumerate(contours):

            ct = time.time()
            local_time = time.localtime(ct)
            date = str(time.strftime("%Y%m%d %H%M%S", local_time))
            micro = datetime.datetime.now().microsecond / 1000  # microsecond
            timestamp = date + str("%03d" % micro)
            x1, y1, w, h = cv2.boundingRect(cnt) # 大框 并非最小矩阵框
            #h1,w1 = template.shape[:2]
            if w * h < AREA_THRESHOLD_L:
                continue
            elif w * h > AREA_THRESHOLD_H:
                continue
            elif w > 220 or h > 220:  #防止检测出长而宽的裂痕
                continue

            if cv2.boundingRect(cnt) != (0, 0, 0, 0):
                crop = img[y1:y1 + h, x1:x1 + w]
            # matchrate = cv2.matchTemplate(gray,template,cv2.TM_CCORR_NORMED)  # 模板匹配
            # #print(matchrate)
            # min_val,max_val,min_loc,max_loc = cv2.minMaxLoc(matchrate)
            # print(min_val,max_val,min_loc,max_loc)
            # loc = np.where(matchrate >= 0.953)
            # for pt in zip(*loc[::-1]):
            #     bottom_right = (pt[0] + w1, pt[1] + h1)
            #     cv2.rectangle(canvas, pt, bottom_right, (255, 0, 0), 1)


            # if np.where(matchrate <= 0.95): # 匹配度大于70% 则留下
            #      continue
            min_area_rect = cv2.minAreaRect(cnt)
            #cv2.contourArea(cnt) # 求面积
            #cv2.arcLength(cnt,True) # 求周长
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            a = float(M["m20"] / M["m00"] - cX * cX)
            b = float(M["m11"] / M["m00"] - cX * cY)
            c = float(M["m02"] / M["m00"] - cY * cY)
            theta = math.atan2(2 * b, (a - c)) / 2  # [-90,90]
            center_x = str(min_area_rect[0][0]).split('.')[0]
            center_y = str(min_area_rect[0][1]).split('.')[0]
            #center_x_real = (int(center_x)-360) / 9.571 only for bottom situation
            #center_y_real = (int(center_y)-180) / 10.29
            #print(center_x_real,center_y_real)
            width = str(min_area_rect[1][0]).split('.')[0]
            height = str(min_area_rect[1][1]).split('.')[0]
            theta1 = str(min_area_rect[2]).split('.')[0]
            #
            # if int(width) > 220 and int(height) < 50: #防止检测出长而宽的裂痕
            #     #print("overlap!!")
            #     continue
            # elif int(width) <50 and int(height) > 220:
            #     continue
            # else:
            #     print("overlap")


            #print(width,height,theta1,"°")

            #HSV color transfer
            if len(crop.shape) == 3:  # enhance effect
                frame = 255 * np.power(crop / 255, 0.5)
                frame = np.around(frame)
                frame[frame > 255] = 255
                frame = frame.astype(np.uint8)
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # change to HSV
            mask_blue = cv2.inRange(img_hsv, color_dict['blue'][0], color_dict['blue'][1])
            mask_green = cv2.inRange(img_hsv, color_dict['green'][0], color_dict['green'][1])
            mask_black = cv2.inRange(img_hsv, color_dict['black'][0], color_dict['black'][1])
            blue_sum = (mask_blue == 255).sum()
            green_sum = (mask_green == 255).sum()
            black_sum = (mask_black == 255).sum()
            sum_list = np.array([blue_sum, green_sum,black_sum])
            res_index = np.argmax(sum_list)  # choose the maximum value
            plate_type = ["blue", "green", "black"]

            center = str(min_area_rect[0][0]).split('.')[0] + ',' + str(min_area_rect[0][1]).split('.')[0]
            x = int(center.split(',')[0])
            y = int(center.split(',')[1])
            yaw = round((min_area_rect[2]), 2)
            rect_contour = np.int64(cv2.boxPoints(min_area_rect))
            #print(left_point_x,right_point_x,top_point_y,bottom_point_y)
            obj_rect_cnt_list.append(rect_contour) #record points of min area rect

            # preference length definition
            if int(width) > int(height):
                pref_long = int(width)
            else:
                pref_long = int(height)
            if int(width) > int(height):
                pref_short = int(height)
            else:
                pref_short = int(width)
            #distance = equ5(pref)

            #record distance according to different colors
            if plate_type[res_index] == 'black':
                #distance = int((f * Wb) / (pref*pi))
                distance = int(equ5(pref_short))
            elif plate_type[res_index] == 'green':
                distance = int(equ6(pref_long))
                # if abs(int(equ5(pref_short)) - distance) > 4:
                #     distance = int(equ5(pref_short))
            elif plate_type[res_index] == 'blue':
                #distance = int((f * W) / (pref*pi))
                distance = int(equ6(pref_long))
                # if abs(int(equ5(pref_short)) - distance) > 4:
                #     distance = int(equ5(pref_short))



            # limit the depth
            if distance < 48:
                distance = 48
            elif distance > 67:
                distance = 67

            # just increase/decrease 1 for each step according to colors
            if plate_type[res_index] == 'blue':
                if abs(last_distance[0] - distance) >= 1:
                    if last_distance[0] > distance:
                        distance = distance + 1
                    elif last_distance[0] < distance:
                        distance = distance - 1
                    else:
                        distance = distance
            elif plate_type[res_index] == 'green':
                if abs(last_distance[1] - distance) >= 1:
                    if last_distance[1] > distance:
                        distance = distance + 1
                    elif last_distance[1] < distance:
                        distance = distance - 1
                    else:
                        distance = distance
            elif plate_type[res_index] == 'black':
                if abs(last_distance[1] - distance) >= 1:
                    if last_distance[2] > distance:
                        distance = distance + 1
                    elif last_distance[2] < distance:
                        distance = distance - 1
                    else:
                        distance = distance
            else:
                continue # if not, skip this contour
            x_pixel = int(equ1(distance))
            y_pixel = int(equ2(distance))
            conver_x = 1
            conver_y = 1
            conver_x = equ3(distance)
            conver_y = equ4(distance)
            center_x_real = int((int(center_x)-x_pixel) / conver_x)
            center_y_real = int((int(center_y)-y_pixel) / conver_y)
            center_xy_real = (center_x_real,center_y_real)
            width_distance = (distance,pref_long,pref_short)
            if plate_type[res_index] == 'black':
                dataline_black = ("black:",distance)
            elif plate_type[res_index] == 'green':
                dataline_green = ("green:",distance)
            elif plate_type[res_index] == 'blue':
                dataline_blue = ("blue:", distance)
            compare_wh = (int(distance),int(width),int(height),int(yaw_actual),int(yaw_actual1))
            #print(center_xy_real)
            print(int(distance),":",center_x,",",center_y,"(",x_pixel,y_pixel,")","(",conver_x,conver_y,")",pref_long, ",", center_x_real,",",center_y_real,",",yaw_actual)
            #print(compare_wh)
            #distance = A*w**2+B*w+C
            #print(distance,"cm",pref) # use shorter length
            #epsilon = 0.01 * cv2.arcLength(cnt, True)
            #approx = cv2.approxPolyDP(cnt, epsilon, True)
            #area_rect = min_area_rect[1][0] * min_area_rect[1][1]
            #area_approx = cv2.contourArea(cnt)
            #cv2.drawContours(canvas, cnt, -1, (255, 0, 0), 2)    # for object contour
            cv2.drawContours(canvas, [rect_contour], -1, (0, 255, 0), 2) # for rectagle contour
            # cv2.circle(canvas, (cX, cY), 4, (0, 255, 255), -1) # for mass center
            cv2.circle(canvas, (int(center.split(',')[0]), int(center.split(',')[1])), 4, (0, 0, 255), -1)
            # cv2.circle(canvas, (int(x2), int(y2)), 4, (0, 0, 255), -1)
            cv2.putText(canvas, center, (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(canvas, str(center_xy_real), (cX - 20, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(canvas, str(compare_wh), (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
                        2)
            # #cv2.putText(canvas, str(dataline_blue), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
            #                 2)
            # cv2.putText(canvas, str(dataline_green), (40, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
            #                 2)
            # cv2.putText(canvas, str(dataline_black), (60, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
            #                 2)
            cvzone.putTextRect(canvas,f'{int(distance)}cm',(x,y+20),thickness=1,scale=1)

            #cv2.arrowedLine(canvas, (70, 70), (1000, 70), color=(0, 255, 0), thickness=2, line_type=8, shift=0,
            #                tipLength=0.05)
            #cv2.arrowedLine(canvas, (70, 70), (70, 746), color=(0, 0, 255), thickness=2, line_type=8, shift=0,
            #                tipLength=0.08)  # coordinate

            # yaw angle function definition
            error = np.rad2deg(theta) - yaw
            if abs(error) <= 20:
                yaw_actual = yaw
            elif abs(error) <= 100:
                if abs(error + 90) <= 20:
                    yaw_actual = yaw - 90

            A = math.tan(np.deg2rad(yaw_actual))

            if abs(yaw_actual) <= 80:
                function_x = 150  # 200 for head right, -200 for head left
            elif 80 <= abs(yaw_actual) <= 88:
                function_x = 10
            elif abs(yaw_actual) <= 80:
                function_x = -150
            elif 80 <= abs(yaw_actual) <= 88:
                function_x = -10
            else:
                function_x = 0
                yaw_actual = yaw_actual

            function_y = A * function_x
            center_x = int(x + function_x)  # start point 2
            center_y = int(y + function_y)  # end point 2
            #print(cX, cY, "vs", int(min_area_rect[0][0]), int(min_area_rect[0][1])) # 看重心和中心的对比
            # for function_x in range(0,200):
            #     if (center_x-x)^2 + (center_y-y)^2 == 500:
            #         function_x = function_x


            if (cX > int(min_area_rect[0][0])) and (cY >= int(min_area_rect[0][1])): # 重心在右下，矩阵中心在左上,方向左上
                #print("重心右下，中心左上")
                cv2.putText(canvas, "重心右下，中心左上", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
                            2)
                yaw_actual1 = int(theta1) - 180
                if yaw_actual > 0:
                    yaw_actual = yaw_actual - 180
                elif yaw_actual > -180 and yaw_actual < -90:
                    yaw_actual = yaw_actual
                #cv2.arrowedLine(canvas, (x, y), (2 * x - center_x, 2 * y - center_y)
                                #, color=(0, 255, 255), thickness=2, line_type=8, shift=0, tipLength=0.05)
            elif (cX > int(min_area_rect[0][0])) and (cY < int(min_area_rect[0][1])): # 重心在右上，矩阵中心在左下
                #print("重心右上，中心左下")
                cv2.putText(canvas, str("重心右上，中心左下"), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
                            2)
                yaw_actual1 = int(theta1) + 90
                if yaw_actual < 0:
                    yaw_actual = yaw_actual + 180
                elif yaw_actual < 180 and yaw_actual > 90:
                    yaw_actual = yaw_actual
                #cv2.arrowedLine(canvas, (x, y), (2 * x - center_x, 2 * y - center_y)
                #                , color=(0, 255, 255), thickness=2, line_type=8, shift=0, tipLength=0.05)
            elif (cX < int(min_area_rect[0][0])) and (cY >= int(min_area_rect[0][1])):  # 重心在左下，矩阵中心在右上
                #print("重心左下，中心右上")
                cv2.putText(canvas, str("重心左下，中心右上"), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
                            2)
                yaw_actual1 = int(theta1) - 90
                #cv2.arrowedLine(canvas, (x, y), (center_x, center_y)
                #               , color=(0, 255, 255), thickness=2, line_type=8, shift=0, tipLength=0.05)
            elif (cX < int(min_area_rect[0][0])) and (cY < int(min_area_rect[0][1])):
                # print("重心左上，中心右下")
                cv2.putText(canvas, str("重心左上，中心右下"), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255),
                            2)
                yaw_actual1 = int(theta1)

            else:
                #print("重心左上，中心右下")
                yaw_actual1 = int(theta1)
                #cv2.arrowedLine(canvas, (x, y), (center_x, center_y)
                #                , color=(0, 255, 255), thickness=2, line_type=8, shift=0, tipLength=0.05)
                if center_y > 600 or center_y < 120:
                    yaw_actual = yaw_actual - 180
            if (180 < yaw_actual <= 270):  # double check
                yaw_actual = yaw_actual - 360
            elif (-270 <= yaw_actual < -180):
                yaw_actual = yaw_actual + 360
            yaw_actual2 = 0
            if yaw_actual > 90 and yaw_actual < 180:
                yaw_actual2 = 270 - yaw_actual
            elif yaw_actual == 90:
                yaw_actual2 = 180
            elif yaw_actual == -90:
                yaw_actual2 = 0
            elif yaw_actual == -180:
                yaw_actual2 = 90
            elif yaw_actual == 0:
                yaw_actual2 = -90
            else:
                yaw_actual2 = -90 - yaw_actual

            log = open('data.txt', mode='a', encoding='utf-8')
            if keyboard.is_pressed('t'):
                switch = 1  # choose if you want to record data
                counter = 0
            elif keyboard.is_pressed('y'):
                switch = 0
            if (counter % 1 == 0) and (switch == 1) : # 帧数速度控制
                print(int(counter),timestamp, plate_type[res_index], center, yaw_actual,distance)
                #print(int(counter),date,micro, plate_type[res_index], center_x, center_y, yaw_actual,
                #      center_x_real, center_y_real, int(distance), file=log)
                print(distance,pref_long,file=log)
                # count, date, time, microsecond, color, x , y , yaw，x_real, y_real distance
                # 前后左右速度和力

            # finally store the value of distance
            if plate_type[res_index] == 'black':
                last_distance[0] = distance
                last_yaw[0] = yaw_actual
            elif plate_type[res_index] == 'green':
                last_distance[1] = distance
                last_yaw[1] = yaw_actual
            elif plate_type[res_index] == 'black':
                last_distance[2] = distance
                last_yaw[2] = yaw_actual
            #print("center = ","(",center,")","yaw = ",yaw_actual,"°") #x,y,yaw
        #cv2.namedWindow("Camera", 0)
        #cv2.resizeWindow("Camera", 1280,720)
        #print(distance,pref)
        cv2.imshow("Camera1", canvas)  # show the screen for camera
        if cv2.waitKey(1) & 0XFF == ord("q"):
            break

        if distance < 50:
            myCmd = str(5)
            myCmd = myCmd + '\r'
            arduinoData.write(myCmd.encode())
        elif distance > 60:
            myCmd = str(4)
            myCmd = myCmd + '\r'
            arduinoData.write(myCmd.encode())

        if counter%4 == 0:
            # center_x_real,center_y_real,yaw_actual
            obs = [[-center_y_real, -center_x_real, yaw_actual2]]

            action = model.predict(obs)
            print("{}".format(action))
            myCmd = str(action[0])
            # print("action: {}".format(cmd))
            myCmd = myCmd + '\r'
            arduinoData.write(myCmd.encode())

    cap.release()
    cv2.destroyAllWindows()
            #print(record_x, record_y, '', x2, y2,'',x,y)
if __name__ == '__main__':
    main()
