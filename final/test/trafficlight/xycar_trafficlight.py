#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from stanley_follower import StanleyController
from TrafficSignDetetector import TrafficSignDetetector
from Stop import Stop

import time

class Xycar(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.stanley_follower = StanleyController()
        self.traffic_light = TrafficSignDetetector()
        self.stop = Stop()
        self.traffic_sign = ""
    
    def control(self) :
        self.stanley_follower.Control()
        self.traffic_sign = self.traffic_light.Detect()
        self.stop.Detect()
        print(self.traffic_sign)
        self.msg.speed = self.stanley_follower.speed
        self.msg.angle = self.stanley_follower.angle        
        
        
        if self.stop.check == True :
            print("정지선")
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(2)
            self.stop.check = False
        
        
        if self.traffic_light.status == True :
            print(self.traffic_sign)
            self.msg.speed = 0
            self.msg.angle = 0
            self.pub.publish(self.msg)
            time.sleep(3)
            self.traffic_light.status = False
        
        #self.msg.speed = 25
        #self.msg.angle = 0
        
        self.pub.publish(self.msg)
        
        self.rate.sleep()
