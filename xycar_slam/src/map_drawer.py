#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
import rospkg
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import pickle


class Drawer(object):
    def __init__(self, file_, r, g, b, a, scale):
        self.file = file_
        self.r = r
        self.g = g
        self.b = b
        self.a = a
        self.scale = scale
        with open(file_, "rb") as f:
            self.path = pickle.load(f)

    def GetMarker(self):
        draw_step = 3

        m = Marker()
        m.id = 1
        m.header.frame_id = "/map"
        m.type = m.LINE_STRIP
        m.action = m.ADD

        m.scale.x = self.scale

        m.color.r = self.r
        m.color.g = self.g
        m.color.b = self.b
        m.color.a = self.a

        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1

        m.points = []

        for j in range(len(self.path["x"])):
            if j % draw_step == 0 or j == len(self.path["x"])-1:
                p = Point()
                p.x = self.path["x"][j]
                p.y = self.path["y"][j]
                m.points.append(p)
        return m

if __name__ == "__main__":
    rospy.init_node("map_rviz_visualizer_node")
    map_file = "/home/nvidia/xycar_ws/src/map/final11.pkl"

    map_pub = rospy.Publisher("/rviz/map", Marker, queue_size=1, latch=True)
    drawer = Drawer(map_file, r=255/255.0, g=236/255.0, b=139/255.0, a=0.8, scale=0.1)
    marker = drawer.GetMarker()

    rospy.sleep(1)
    while not rospy.is_shutdown():
        map_pub.publish(marker)
        rospy.sleep(1)
