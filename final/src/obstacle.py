#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan

lidar_points = None

class Obstacle(object) :

  def __init__(self):
    self.sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
    self.obstacle_state = 'go'
  
  
  def lidar_callback(self, data):
    global lidar_points
    lidar_points = data.ranges

  def Detect(self) :
    if lidar_points == None:
      return "no lidar data"
    
    left = lidar_points[:90]
    right = lidar_points[415:505]
    left_count = 0
    right_count = 0
  
  
    for i in left :
      if 0 < i < 0.6 :
        left_count += 1
  
    for i in right :
      if 0 < i < 0.6 :
        right_count += 1
      
    if left_count < right_count and 20 > right_count > 4 :
      self.obstacle_state = "left turn"
    elif left_count >= right_count and 20 > left_count > 4 :
      self.obstacle_state = "right turn"
    else :
      self.obstacle_state = "go straight"
    print("left_count", left_count)
    print("right_count", right_count)
    