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
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.lidar_points = None
        self.fm_list = None
        self.fr_list = None
        self.front_count = 0
        self.right_count = 0
        self.front_flag = False
        self.right_flag = False
        self.end_flag = False

  
    def lidar_callback(self, data):
        self.lidar_points = data.ranges


    def Detect(self):
        if self.lidar_points == None:
              return

        self.fm_list = self.lidar_data[-25:]+self.lidar_data[:25]
        self.fm_list = list(self.fm_list)
        self.fm_list[:] = [value for value in self.fm_list if value != 0]
        self.fr_list = self.lidar_data[-75:-60]
        self.fr_list = list(self.fr_list)
        self.fr_list[:] = [value for value in self.fr_list if value != 0]

        self.front_count = 0
        self.right_count = 0
    
        for i in self.fm_list:
            if 0.3 < i < 0.5:
                self.front_count += 1

        for i in self.fr_list:
            if 0.3 < i < 0.5:
                self.right_count += 1

        self.front_flag = False
        self.right_flag = False

        if self.front_count > :
            self.front_flag = True

        if self.right_count > 8:
            self.right_flag = True
        


