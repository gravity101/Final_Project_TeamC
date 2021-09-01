#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal


image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
WIDTH = 640
HEIGHT = 480


calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))


def calibrate_image(frame):
    global WIDTH, HEIGHT
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (WIDTH, HEIGHT))


def find_circle(v):
    range_L = cv2.inRange(v, 220, 255)

    # 신호 부분 원으로 표시
    circles = cv2.HoughCircles(range_L, cv2.HOUGH_GRADIENT, 1, 100, param1=250, param2=10, minRadius=10, maxRadius=30)
    if circles is not None:
        i = circles[0][0]
        cv2.circle(roi, (i[0], i[1]), i[2], (0, 0, 255), 5)

    # 원의 중심 표시
    x = circles[0][0][0]
    y = circles[0][0][1]
    cv2.circle(roi, (x, y), 2, (0, 0, 0), 5)

    return x, y
    

def find_rect(v):
    # threshold 후 윤곽선 찾기
    ret, thr = cv2.threshold(v, 180, 255, 0)
    _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(contours, key = cv2.contourArea, reverse = True)

    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02*peri, True)

        if len(approx) == 4:
            cnt_width_max = max(approx[0][0][0], approx[1][0][0], approx[2][0][0], approx[3][0][0])
            cnt_width_min = min(approx[0][0][0], approx[1][0][0], approx[2][0][0], approx[3][0][0])
            cnt_height_max = max(approx[0][0][1], approx[1][0][1], approx[2][0][1], approx[3][0][1])
            cnt_height_min = min(approx[0][0][1], approx[1][0][1], approx[2][0][1], approx[3][0][1])
            cnt_width = cnt_width_max - cnt_width_min
            cnt_height = cnt_height_max - cnt_height_min

	# 사각형 표시
        if len(approx) == 4 and cnt_width < 120 and cnt_width > 80:
            screenCnt = approx
            cv2.drawContours(roi, [screenCnt], -1, (0, 255, 0), 2)

            # 사각형의 중심 표시
            moment = cv2.moments(c)
            x = int(moment['m10']/moment['m00'])
            y = int(moment['m01']/moment['m00'])
            cv2.circle(roi, (rect_x, rect_y), 2, (0, 0, 0), 5)

    cv2.imshow('Outline', roi)
    return x, y, cnt_width_max, cnt_width_min, cnt_height_max, cnt_height_min, cnt_width, cnt_height


def light_decide(circle_x, circle_y, cwmax, cwmin, chmax, chmin, cw, ch):

    if circle_x < cwmin:
        print("Outside")
        drive(0, 0)
    elif circle_x > cwmax:
        print("Outside")
        drive(0, 0)
    elif circle_y < chmin:
        print("Outside")
        drive(0, 0)
    elif circle_y > chmax:
        print("Outside")
        drive(0, 0)
    elif circle_x < cwmin + cw/3:
        print("Red")
        drive(0, 0)
    elif circle_x > cwmax - cw/3:
        print("Green")
        drive(0, 0)
    else:
        print("Yellow")
        drive(0, 0)


# 카메라로부터 Image형 토픽이 오면 전처리 시킬 수 있는 이미지로 변환 작업
def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# 각도와 속도를 모터 노드로 publish
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)


def main():
    global pub
    global image
    global cap
    global WIDTH, HEIGHT

    rospy.init_node('traffic_light')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print("---------- Xycar A2 v1.0 ----------")
    rospy.sleep(2)

    while True:

        while not image.size == (WIDTH*HEIGHT*3):
            continue

        image = calibrate_image(image)

        # ROI 및 HSV 전처리
        roi = img[:120, 160:400]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        circle_x, circle_y = find_circle(v)
        rect_x, rect_y, cwmax, cwmin, chmax, chmin, cw, ch = find_rect(v)
        light_decide(circle_x, circle_y, cwmax, cwmin, chmax, chmin, cw, ch)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()


if __name__ == '__main__':
    main()


