#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from tuning_stanley_follower import StanleyController
from Stop import Stop

import time

class Xycar(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.stanley_follower = StanleyController()
        self.stop = Stop()
    
    def control(self) :
        self.stanley_follower.Control()
        self.stop.Detect()
        self.msg.speed = self.stanley_follower.speed
        self.msg.angle = self.stanley_follower.angle
        
        if self.stop.check == True :
            print("정지선")
            self.msg.speed = 0
            self.msg.angle = 0
            time.sleep(2)
            self.pub.publish(self.msg)
            self.stop.check = False

        #self.msg.speed = 0
        #self.msg.angle = 0
        self.pub.publish(self.msg)
        
        self.rate.sleep()
