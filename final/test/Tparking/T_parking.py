#! /usr/bin/env python
#-*- coding: utf-8 -*-
import rospy, math
import cv2, time, rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


class T_parking():
    def __init__(self):
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.Tparking_time = time.time()
        
        self.speed = 0.0
        self.angle = 0.0
        self.atan = 0
        self.temp = 99
        self.flag1 = False
        self.flag2 = False
        self.flag_end = True
        self.stablize_num = 0
        self.lidar_points = None
        self.lidar_check = False
        self.ar_check = False
        self.arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}


    def ar_callback(self, msg):
        
        for i in msg.markers:
            self.arData["DX"] = i.pose.pose.position.x
            self.arData["DY"] = i.pose.pose.positionkk.y
            self.arData["DZ"] = i.pose.pose.position.z
    
            self.arData["AX"] = i.pose.pose.orientation.x
            self.arData["AY"] = i.pose.pose.orientation.y
            self.arData["AZ"] = i.pose.pose.orientation.z
            self.arData["AW"] = i.pose.pose.orientation.w

            self.temp = i.id

        self.ar_check = True
        self.atan = math.degrees(math.atan2(self.arData["DX"], self.arData["DY"]))
    
    
    def lidar_callback(self, data):
        self.lidar_points = data.ranges
        if self.lidar_points != None:
            self.lidar_check = True
        else:
            self.lidar_check = False
      

    def Detect(self):

        if not self.lidar_check:
            return 0
        else:
          # 기둥을 라이다로 인식
          count1 =0
          for i in range(0,31):      
              if (0.9 <= self.lidar_points[i] <= 1.04):
                  count1 +=1
                  if (count1 >= 15):
                      self.flag1 =True
        
          count2=0
          for i in range(475,505):      
              if (0.9 <= self.lidar_points[i] <= 1.04):
                  count2 +=1
                  if (count2 >= 15):
                      self.flag1 =True
                
          count =0 
          for i in range(30,51):      
              if (1.64 <= self.lidar_points[i] <= 1.72):
                  count +=1
                  if (count >= 5):
                      self.flag2 =True


    def Action(self):
        ################################################################
        #기둥이 인식되면 조금 전진 후 멈췄다가 ar 태그가 일정 거리가 될때까지 후진
        if(self.flag1 and self.flag2):
            self.Tparking_time = time.time() + 1.8
            while(time.time() < self.Tparking_time):
                self.msg.angle = 0
                self.msg.speed = 20
                self.pub.publish(self.msg)

            self.Tparking_time = time.time() + 2.0
            while(time.time() < self.Tparking_time):
                self.msg.angle = 0
                self.msg.speed = 0
                self.pub.publish(self.msg)

            for j in range(11):
                print("backward", j)
                self.msg.angle = -50
                self.msg.speed = -30
                self.pub.publish(self.msg)
                time.sleep(0.5)
            print("backward over")
            

    def Stablize(self):

        if(self.flag1 and self.flag2):
            if self.arData["DZ"] < 0.7:
                self.msg.angle = -50
                self.msg.speed = -30
                self.pub.publish(self.msg)
            elif self.arData["DZ"] > 0.7:
                self.atan = math.degrees(math.atan2(self.arData["DX"], self.arData["DY"]))

                if abs(self.atan)>9:
                    if (self.arData["DZ"] >0.5):
                        self.msg.angle = self.atan*0.3 -30
                        self.msg.speed = 20
                        self.pub.publish(self.msg)
                        time.sleep(0.5)
      
                    if (self.arData["DZ"] < 0.5):
                        self.msg.angle = 0
                        self.msg.speed = -30        
                        self.pub.publish(self.msg)
                        time.sleep(0.5)


