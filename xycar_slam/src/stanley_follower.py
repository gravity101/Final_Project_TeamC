#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor


class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.v = 0
        self.imu_data = -0.98
        self.speed = 0
        self.angle = 0
        self.stanley = StanleyController()
        
        with open("/home/nvidia/xycar_ws/src/map/final_0903_half.pkl", "rb") as f:
            self.path = pickle.load(f)
        
        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        self.v = 30
        

    def GetMsg(self):
      delta, self.present_time = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v, 
                               self.path['x'], self.path['y'], self.path['yaw'], 
                               0.1, 50)
      if delta < -21.155:
        delta = -21.155
      elif delta > 21.155:
        delta = 21.155

      angle = int(delta * 50 / 21.1549)
      
      if angle <-50:
        angle=-50
      elif angle >50:
        angle = 50
        
      self.angle = angle
      self.speed = 25

    def Control(self) :
        x = self.stanley.rear_x
        y = self.stanley.rear_y
        self.stanley.GetMsg()