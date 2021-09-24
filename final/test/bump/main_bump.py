#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from xycar_bump import Xycar

rospy.init_node('final_main')

xycar = Xycar()

while not rospy.is_shutdown():
    xycar.control()
