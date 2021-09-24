#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from stanley_follower import StanleyController
from TrafficSignDetetector import TrafficSignDetetector
from obstacle import Obstacle
from Stop import Stop
from bump import Bumper
from Tparking_test import T_parking

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
        self.obstacle = Obstacle()
        self.tparking = T_parking()

        self.traffic_sign = ""
        
    
    def control(self) :
        
        self.stanley_follower.Control()
        self.traffic_sign = self.traffic_check.Detect()
        self.stop.Detect()
        self.bump.BumpAction()
        self.obstacle.Detect()
        
        if self.stanley_follower.index > 12500 and self.stanley_follower.index < 30000:
            self.tparking.Detect()

        #print(self.traffic_sign)

        # Tracking(Stanley Method)가 속도와 조향각의 기본 베이스로 들어간다.
        self.msg.speed = self.stanley_follower.speed
        self.msg.angle = self.stanley_follower.angle
        
        # 방지턱이 없는 경우 bump_speed는 0으로 아무런 영향이 없고,
        # Imu센서를 통해 방지턱을 감지한 경우에 bump_speed가 15와 -25 등으로 바뀌며 속도에 영향을 주고, 조향각은 0이 된다.
        self.msg.speed += self.bump.bump_speed
        if self.bump.bump_speed != 0:
            self.msg.angle = 0
        
        # 정지선을 보면 2초 간 정지
        if self.stop.check == True and self.traffic_sign == "Nothing" :
            #print("정지선")
            self.msg.speed = 0
            self.msg.angle = 0
            time.sleep(2)
            self.pub.publish(self.msg)
            self.stop.check = False

        # 빨간불이나 노란불에서 5초 간 정지 (5초 미만에서는 불이 바뀔 때 출발해버림)
        if self.traffic_check.status == True and (self.stanley_follower.index < 15000 or self.stanley_follower.index > 35000):
            print(self.traffic_sign)
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(5)
            self.traffic_check.status = False
        
        
        if self.obstacle.obstacle_state == "left turn" and self.stanley_follower.index > 30000 :
          self.msg.angle = -50
        elif self.obstacle.obstacle_state == "right turn" and self.stanley_follower.index > 30000 :
          self.msg.angle = 50
        
        #print(self.obstacle.obstacle_state)

        
        if self.tparking.flag1 and self.tparking.flag2 and self.tparking.flag_end:
            self.tparking.Action()
                
            for i in range(5):
                print("Stablize", i)
                self.tparking.Stablize()

            self.tparking.flag1 = False
            self.tparking.flag2 = False
            self.tparking.flag_end = False

        """        
        if self.tparking.flag1 and self.tparking.flag2 and self.tparking.flag_end:
            self.tparking.Action()
            self.tparking.stablize_num = 1

            self.tparking.flag1 = False
            self.tparking.flag2 = False

        if (1 <= self.tparking.stablize_num <= 5)    
            self.tparking.Stablize()
            self.tparking.stablize_num += 1

            if self.tparking.stablize_num == 5:
                self.tparking.flag_end = False
                
        """


        #self.msg.speed = 0
        #self.msg.angle = 0
        self.pub.publish(self.msg)
        
        self.rate.sleep()


