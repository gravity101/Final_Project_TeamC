#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
import time
import cv2

from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu,Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge

class Bumper(object):
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.imu_data = None
        self.bump_check = 0
        self.bump_time = time.time()
        self.bump_speed = 0

        self.imu_sub = rospy.Subscriber("imu", Imu, self.callback_imu)

    def callback_imu(self, msg):        
        self.imu_data = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])

        #print('Roll:%.4f, Pitch:%.4f, Yaw:%.4f' % (self.roll, self.pitch, self.yaw))

    def BumpAction(self):
        
        if self.imu_data == None:
            return 0

        # 방지턱을 감지하면 check = 1
        if self.pitch < 0 and self.bump_check == 0:
            self.bump_check = 1
            self.bump_time = time.time() + 1.6

        # 1.7초 간 속도 40으로 주행 후 check = 2
        if self.bump_check == 1:
            if self.bump_time > time.time():
                self.bump_speed = 15
            else:
                self.bump_check = 2
                self.bump_time = time.time() + 4.0

        # 3.0초 간 정지 후 check = 0
        if self.bump_check == 2:
            if self.bump_time > time.time():
                self.bump_speed = -30
            else:
                self.bump_speed = 0
                self.bump_check = 0
