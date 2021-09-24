#! /usr/bin/env python
#-*- coding: utf-8 -*-
import rospy, math
import cv2, time, rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


class YOLO():
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback_box)
        
        self.boxdata = None
        self.flag_seq = 0
        self.flag_person = False
        self.flag_cat = False


    def callback_box(self, msg):
        self.boxdata = msg

        if self.boxdata is not None:
            for i in self.boxdata.bounding_boxes:
                if i.Class == "person" and flag_seq == 1 and not flag_person
                    flag_person = True
                    flag_seq = 2
                    break
                if i.Class == "cat" and flag_seq == 2 and not flag_cat:
                    flag_cat = True
                    flag_seq = 3
                    break
    

