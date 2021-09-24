#! /usr/bin/env python
#-*- coding: utf-8 -*-
import rospy, math
import cv2, time, rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32MultiArray
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor

from stanley_follower import StanleyController


class YOLO():
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback_box)
        
        self.boxdata = None
        self.flag_seq = 0
        self.flag_person = False
        self.flag_cat = False

        self.sf = StanleyController()

    def callback_box(self, msg):
        self.boxdata = msg

        if self.boxdata is not None:
            for i in self.boxdata.bounding_boxes:
                if i.Class == "person":
                    print(self.sf.index)
                if i.Class == "person" and self.flag_seq == 1 and not self.flag_person:
                    self.flag_person = True
                    self.flag_seq = 2
                    break
                if i.Class == "cat" and self.flag_seq == 2 and not self.flag_cat:
                    self.flag_cat = True
                    self.flag_seq = 3
                    break
    

