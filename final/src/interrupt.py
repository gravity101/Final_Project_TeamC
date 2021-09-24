#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

lidar_points = None
ultra = [0,0,0,0,0]

class interrupt() :

  def __init__(self):
    rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, self.ultra_callback)
    self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    self.msg = xycar_motor()
    self.speed = 0.0  
    self.inter_flag = False
  
  def lidar_callback(self, data):
    global lidar_points
    lidar_points = data.ranges

  def ultra_callback(self, msg):
    global ultra
    for j in msg.data:
      #print(msg.data)
      #print(msg.data[0])
      #left
      ultra[0] = msg.data[0]
      #right
      ultra[1] = msg.data[4]
      #rear right
      ultra[2] = msg.data[5]
      #rear mid
      ultra[3] = msg.data[6]
      #rear left
      ultra[4] = msg.data[7]

  def cross(self):
    global ultra
    if lidar_points == None:
      return "no lidar data"

    left = lidar_points[:180]
    #right = lidar_points[379:505]
    left_count = 0
    #right_count = 0    
    
    for i in left :
      if 0 < i < 0.1 :
        left_count += 1
        
        
    if left_count > 10:
      print("lidar")
    if ultra[0]< 40 or ultra[4] < 40:
      print("sonic")
    
    # 라이더 좌측에 가깝게 있거나 좌측 초음파, 후방 좌측 초음파에 가까우면 정지
    if left_count > 10 or ultra[0]< 40 or ultra[4] <40:
        print("wait!")
        self.inter_flag = True
        self.msg.speed = 0
        self.msg.angle = 0
        self.pub.publish(self.msg)
        rospy.sleep(3)
    else:
        self.inter_flag = False
        
