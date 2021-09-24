#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 

WIDTH, HEIGHT = 640, 480

class Stop(object):

    def __init__(self) :
        global WIDTH, HEIGHT
        self.image = np.empty(shape=[0])
        self.calibration = np.empty(shape=[0])
        self.check = False
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        # calibration 변수
        self.mtx = np.array([
            [422.037858, 0.0, 245.895397], 
            [0.0, 435.589734, 163.625535], 
            [0.0, 0.0, 1.0]
        ])
        self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))


    def calibrate_image(self, frame, mtx, dist, cal_mtx, cal_roi):
        tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
        x, y, w, h = cal_roi
        tf_image = tf_image[y:y+h, x:x+w]
        return cv2.resize(tf_image, (frame.shape[1], frame.shape[0]))

    def img_callback(self, data) :   
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data, "bgr8")
            
        
    def process_image(self, data):
        blur = cv2.GaussianBlur(data, (5, 5), 0)
        imgray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
            
        _, lane = cv2.threshold(imgray, 150, 200, cv2.THRESH_BINARY_INV)
        #cv2.imshow('lane', lane)
        _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            
        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)
            x, y, w, h = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            _, width, _ = data.shape
            #cv2.waitKey(1)
            if (200 <= center[0] <= (width - 200)) and  240 <= center[1] <= 400 and (w > 200) & (h < 80) :
                cv2.rectangle(data, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.check = True
            
        #cv2.imshow('contour', data)
        #cv2.waitKey(1)
            
    def Detect(self) :
        global WIDTH, HEIGHT

        while not self.image.size == (WIDTH*HEIGHT*3):
            return "No image"

        self.calibration = self.calibrate_image(self.image, self.mtx, self.dist, self.cal_mtx, self.cal_roi)
        self.process_image(self.calibration)
