#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


class test() :

    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.callback_lidar, queue_size=1)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.lidar_points = None
        self.fr_list = None
        self.lidar_count = 0
        self.inter_flag = False

  
    def callback_lidar(self, data):
        self.lidar_points = data.ranges


    def Detect(self):
        print("==================================================================================")
        if self.lidar_points == None:
            return

        self.fr_list = self.lidar_points[-75:-60]
        self.fr_list = list(self.fr_list)

        for i in range(len(self.fr_list)):
            print("index: ", i+430, ", lidar: ", self.fr_list[i])


#    self.fr_list[:] = [value for value in self.fr_list if value != 0]        

