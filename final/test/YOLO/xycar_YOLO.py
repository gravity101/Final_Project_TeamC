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
from YOLO import YOLO

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
        self.yolo = YOLO()

        self.traffic_sign = ""
        
    
    def control(self) :
        
        self.stanley_follower.Control()
        self.traffic_sign = self.traffic_check.Detect()
        self.stop.Detect()
        self.bump.BumpAction()
        self.obstacle.Detect()

        print(self.traffic_sign)

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
            print("정지선")
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
                

        if self.stanley_follower.index > 14000 and self.stanley_follower.index < 18000:
            self.yolo.flag_seq = 1
            if self.yolo.flag_person:
                # False로 안 바꿔주면 다음 루프에서 또 멈춤. 어차피 flag_seq가 늘어나서 다시 True로 바뀔 일은 없음
                self.yolo.flag_person = False
                time.sleep(5)
            if self.yolo.flag cat:
                self.yolo.flag_cat = False
                time.sleep(5)

                

        #self.msg.speed = 0
        #self.msg.angle = 0
        self.pub.publish(self.msg)
        
        self.rate.sleep()

