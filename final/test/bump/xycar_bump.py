#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from stanley_follower import StanleyController
from TrafficSignDetetector import TrafficSignDetetector
from Stop import Stop
from bump import Bumper

import time

class Xycar(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.stanley_follower = StanleyController()
        self.traffic_check = TrafficSignDetetector()
        self.stop = Stop()
        self.bump = Bumper()
        self.traffic_sign = ""
    
    def control(self) :
        self.stanley_follower.Control()
        self.traffic_sign = self.traffic_check.Detect()
        self.stop.Detect()
        self.bump.BumpAction()

        # Tracking(Stanley Method)가 속도와 조향각의 기본 베이스로 들어간다.
        self.msg.speed = self.stanley_follower.speed
        self.msg.angle = self.stanley_follower.angle

        # 방지턱이 없는 경우 bump_speed는 0으로 아무런 영향이 없고,
        # Imu센서를 통해 방지턱을 감지한 경우에 bump_speed가 15와 -25 등으로 바뀌며 속도에 영향을 주고, 조향각은 0이 된다.
        self.msg.speed += self.bump.bump_speed
        if self.bump.bump_speed != 0:
            self.msg.angle = 0
        
        if self.stop.check == True :
            print("정지선")
            self.msg.speed = 0
            self.msg.angle = 0
            time.sleep(2)
            self.pub.publish(self.msg)
            self.stop.check = False
        
        
        #self.msg.speed = 25
        #self.msg.angle = 0
        
        self.pub.publish(self.msg)
        
        self.rate.sleep()
