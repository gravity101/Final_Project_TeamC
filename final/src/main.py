#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from xycar import Xycar

rospy.init_node('final_main')

xycar = Xycar()

while not rospy.is_shutdown():
    xycar.control()