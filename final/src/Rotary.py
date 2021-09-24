#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor


class rotary() :

    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback_box)
#        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
#        rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, self.ultra_callback)

        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.lidar_points = None
        self.boxdata = None
        self.location = -1
        self.flag_rotary = False


    def callback_box(self, msg):
        self.boxdata = msg

        if self.boxdata is not None:
            for i in self.boxdata.bounding_boxes:
                if i.Class == "car":
                    self.location = (i.xmin + i.xmax) // 2
                    break


