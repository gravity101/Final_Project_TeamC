#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import sys
import os
import signal


bridge = CvBridge()
WIDTH = 640
HEIGHT = 480


class TrafficSignDetetector() :

    def __init__(self) :
        self.status = False	# 멈춰야할 때 (빨간불이나 노란불일 때) True
        self.image = np.empty(shape=[0])
        self.roi = np.empty(shape=[0])
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # calibration 변수
        self.mtx = np.array([
            [422.037858, 0.0, 245.895397], 
            [0.0, 435.589734, 163.625535], 
            [0.0, 0.0, 1.0]
        ])
        self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))
    
    # 카메라로부터 Image형 토픽이 오면 전처리 시킬 수 있는 이미지로 변환 작업
    def img_callback(self, data):
        self.image = bridge.imgmsg_to_cv2(data, "bgr8")

    def calibrate_image(self, frame):
        global WIDTH, HEIGHT
        
        tf_image = cv2.undistort(frame, self.mtx, self.dist, None, self.cal_mtx)
        x, y, w, h = self.cal_roi
        tf_image = tf_image[y:y+h, x:x+w]

        return cv2.resize(tf_image, (WIDTH, HEIGHT))


    def find_circle(self, v):
        range_L = cv2.inRange(v, 220, 255)

        # 신호 부분 원으로 표시
        circles = cv2.HoughCircles(range_L, cv2.HOUGH_GRADIENT, 1, 200, param1=250, param2=10, minRadius=7, maxRadius=20)
        if circles is not None:
            i = circles[0][0]
            cv2.circle(self.roi, (i[0], i[1]), i[2], (0, 0, 255), 5)

            # 원의 중심 표시
            x = circles[0][0][0]
            y = circles[0][0][1]
            cv2.circle(self.roi, (x, y), 2, (0, 0, 0), 5)

            return x, y

        return 0, 0
        

    def find_rect(self, rgb, v):
        kernel = np.ones((3, 3), np.uint8)

        range_R = cv2.inRange(rgb, (0, 0, 0), (125, 125, 125))

        cv2.imshow("range", range_R)
    
        erosion = cv2.erode(range_R, kernel, iterations=1)
        dilation = cv2.dilate(erosion, kernel, iterations=1)
        _, contours, _ = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            cnts = sorted(contours, key=cv2.contourArea, reverse=True)
            cv2.drawContours(self.roi, [cnts[0]], 0, (255, 0, 0), 2)
            x = []
            y = []
            for i in range(len(cnts[0])):
                x.append(cnts[0][i][0][0])
                y.append(cnts[0][i][0][1])
            x_min = min(x)
            x_max = max(x)
            y_min = min(y)
            y_max = max(y)

            moment = cv2.moments(cnts[0])
            rect_x = int(moment['m10'] / moment['m00'])
            rect_y = int(moment['m01'] / moment['m00'])
            cv2.circle(self.roi, (rect_x, rect_y), 2, (0, 0, 0), 5)

#            cv2.imshow('Outline', self.roi)
            return rect_x, rect_y, x_max, x_min, y_max, y_min, x_max - x_min, y_max - y_min

        return 0, 0, 0, 0, 0, 0, 0, 0


    def light_decide(self, circle_x, circle_y, cwmax, cwmin, chmax, chmin, cw, ch):
        
        if circle_x + circle_y == 0 or cwmax + cwmin == 0:
            # 신호나 신호등이 존재하지 않는 경우
            self.status = False
            return "Nothing"
        elif circle_x < cwmin:
            # 동그라미와 사각형은 있지만 신호등이 아닌 경우
            self.status = False
            return "Outside"
        elif circle_x > cwmax:
            # 동그라미와 사각형은 있지만 신호등이 아닌 경우
            self.status = False
            return "Outside"
        elif circle_y < chmin:
            # 동그라미와 사각형은 있지만 신호등이 아닌 경우
            self.status = False
            return "Outside"
        elif circle_y > chmax:
            # 동그라미와 사각형은 있지만 신호등이 아닌 경우
            self.status = False
            return "Outside"
        elif circle_x < cwmin + cw/3:
            self.status = True
            return "Red"
        elif circle_x > cwmax - cw/3:
            self.status = False
            return "Green"
        else:
            self.status = True
            return "Yellow"


    def Detect(self):
        global cap
        global WIDTH, HEIGHT

        while not self.image.size == (WIDTH*HEIGHT*3):
            return "No image"

        # ROI 및 HSV 전처리
        self.roi = self.calibrate_image(self.image)[40:150, 190:370]
        hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        circle_x, circle_y = self.find_circle(v)
        rect_x, rect_y, cwmax, cwmin, chmax, chmin, cw, ch = self.find_rect(self.roi, v)
        sign = self.light_decide(circle_x, circle_y, cwmax, cwmin, chmax, chmin, cw, ch)

#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break

        return sign
