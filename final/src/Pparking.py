#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


class pparking() :

    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.callback_lidar, queue_size=1)
        rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, self.ultra_callback)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        
        self.msg = xycar_motor()

        self.lidar_points = None
        self.fm_list = None
        self.fr_list = None
        self.over_list = None
        self.front_count = 0
        self.right_count = 0
        self.over_count = 0
        self.front_flag = False
        self.right_flag = False
        self.over_flag = False
        self.end_flag = False
        self.forward = 0
        self.sonic_flag = False
        self.ultra = [0,0,0,0,0]
        
    def ultra_callback(self, msg):
      for j in msg.data:
        #print(msg.data)
        #print(msg.data[0])
        #left
        self.ultra[0] = msg.data[0]
        #right
        self.ultra[1] = msg.data[4]
        #rear right
        self.ultra[2] = msg.data[5]
        #rear mid
        self.ultra[3] = msg.data[6]
        #rear left
        self.ultra[4] = msg.data[7]

  
    def callback_lidar(self, data):
        self.lidar_points = data.ranges


    def Detect(self):
        if self.lidar_points == None:
              return

        self.fm_list = self.lidar_points[-20:]+self.lidar_points[:20]
        self.fm_list = list(self.fm_list)
        self.fm_list[:] = [value for value in self.fm_list if value != 0]

        self.fr_list = self.lidar_points[-105:-60]
        self.fr_list = list(self.fr_list)
        self.fr_list[:] = [value for value in self.fr_list if value != 0]

        self.over_list = self.lidar_points[-60:-1]
        self.over_list = list(self.over_list)
        self.over_list[:] = [value for value in self.fr_list if value != 0]

        self.front_count = 0
        self.right_count = 0
        self.over_count = 0
    
        for i in self.fm_list:
            if 0.1 < i < 0.5:
                self.front_count += 1

        for i in self.fr_list:
            if 0.2 < i < 0.8:
                self.right_count += 1

        for i in self.over_list:
            if 0.4 < i < 0.85:
                self.over_count += 1

        self.front_flag = False
        self.right_flag = False
        self.over_flag = False
        self.sonic_flag = False


        if self.front_count > 8:
            print("front_count: ", self.front_count)
            self.front_flag = True

        if self.right_count > 7:
            print("right_count: ", self.right_count)
            self.right_flag = True
        
        if self.over_count > 10:
            print("over_count: ", self.over_count)
            self.over_flag = True
            
        if self.ultra[1] < 30:
            print("sonic")
            self.sonic_flag = True
        


