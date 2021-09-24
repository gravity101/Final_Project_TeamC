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
from interrupt import interrupt
from Tparking import T_parking
from Pparking import pparking

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
        self.interrupter = interrupt()
        self.pparking = pparking()
        self.interrupt = True
        self.yolo_stop1 = True
        self.yolo_stop2 = True
        
        self.traffic_sign = ""
        
    
    def control(self) :
        
        self.stanley_follower.Control()
        self.traffic_sign = self.traffic_check.Detect()
        self.stop.Detect()
        self.bump.BumpAction()
        self.obstacle.Detect()
        self.pparking.Detect()

        if (4100 < self.stanley_follower.index < 4800):
            self.tparking.Detect()

        # Tracking(Stanley Method)가 속도와 조향각의 기본 베이스로 들어간다.
        self.msg.speed = self.stanley_follower.speed
        self.msg.angle = self.stanley_follower.angle
        """
        print(self.stanley_follower.index)
        if (700 < self.stanley_follower.index < 1600):
            self.interrupter.cross()
            #if self.interrupter.inter_flag == True:
            print(self.stanley_follower.yaw)
            if(self.interrupter.inter_flag == True and self.stanley_follower.yaw < 0.2):
                self.msg.speed = self.interrupter.speed
                print("inter_flag == True")
                #rospy.sleep(3)
            else:
                self.msg.speed = self.stanley_follower.speed
                self.msg.angle = self.stanley_follower.angle
                print("inter_flag == False")
        """
        
        if (880 < self.stanley_follower.index < 1000) and self.interrupt:
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(0.8)
            self.interrupt = False

        # 빨간불이나 노란불에서 5초 간 정지 (5초 미만에서는 불이 바뀔 때 출발해버림)
        if self.traffic_check.status == True and (self.stanley_follower.index < 400):
            print(self.traffic_sign)
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(7)
            self.traffic_check.status = False
        
        if (4600 < self.stanley_follower.index < 4800) and self.tparking.flag1 and self.tparking.flag2 and self.tparking.flag_end:
            self.tparking.Action()
            """    
            while(self.tparking.flag_sta == False):
                self.tparking.Stablize()
            """
            self.tparking.flag1 = False
            self.tparking.flag2 = False
            self.tparking.flag_end = False

        if (4900 < self.stanley_follower.index < 4940) and self.yolo_stop1:  # 장기용
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(5)
            self.yolo_stop1 = False
        if (5510 < self.stanley_follower.index < 5550) and self.yolo_stop2:  # 고양이
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(5)
            self.yolo_stop2 = False
            
        # 방지턱이 없는 경우 bump_speed는 0으로 아무런 영향이 없고,
        # Imu센서를 통해 방지턱을 감지한 경우에 bump_speed가 15와 -25 등으로 바뀌며 속도에 영향을 주고, 조향각은 0이 된다.
        if (9200 < self.stanley_follower.index < 10130):
            self.msg.speed += self.bump.bump_speed
        if self.bump.bump_speed != 0:
            self.msg.angle = 0
        
        # 정지선을 보면 2초 간 정지
        if self.stop.check == True and (9200 < self.stanley_follower.index < 10130):
            print("정지선")
            time.sleep(2)
            self.msg.speed = 20
            self.msg.angle = 0
            self.pub.publish(self.msg)
            self.stop.check = False

        if self.obstacle.obstacle_state == "left turn" and (11000 < self.stanley_follower.index < 12000):
            self.msg.angle = -40
        elif self.obstacle.obstacle_state == "right turn" and (11000 < self.stanley_follower.index < 12000):
            self.msg.angle = 40
        
        if self.pparking.front_flag and self.pparking.right_flag and self.stanley_follower.index > 12300:
            print("Over")
            self.msg.angle = 0
            self.msg.speed = 0
            self.pub.publish(self.msg)
            time.sleep(1000)            
            return      

        if (self.pparking.sonic_flag or self.pparking.right_flag) and not self.pparking.front_flag and not self.pparking.over_flag and self.stanley_follower.index > 12300:
            print("앞으로")
            #self.pparking.forward += 1
            self.msg.angle = self.stanley_follower.yaw_term
            self.msg.speed = 20
            self.pub.publish(self.msg)
            time.sleep(3)
            return 
        
        #self.msg.speed = 0
        #self.msg.angle = 0
        self.pub.publish(self.msg)
        
        self.rate.sleep()
