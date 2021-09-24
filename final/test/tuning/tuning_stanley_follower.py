#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
from tuning_stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.v = 0
        self.speed = 0
        self.angle = 0
        self.index = 0
        
        with open("/home/nvidia/xycar_ws/src/map/final_0910.pkl", "rb") as f:
            self.path = pickle.load(f)
        
        self.tracked_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)

    def ImuCallBack(self, msg):        
        self.imu_data = msg.linear_acceleration.z
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        #print("yaw: ", self.yaw)

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        """
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        """
        self.v = 30
        

    def Control(self):
      	self.yaw -=2.29
        print("yaw: ", self.yaw)
        print("map_yaw: ", self.path['yaw'][0])
        print("---------------------------")
        delta, idx = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v, self.index, 
                               self.path['x'], self.path['y'], self.path['yaw'], 0.12, 50)

        self.index = idx

        if delta < -21.155:
            delta = -21.155
        elif delta > 21.155:
            delta = 21.155

        angle = int(delta * 50 / 21.1549)
      
        if angle < -50:
            angle = -50
        elif angle > 50:
            angle = 50
        
        self.angle = angle
        self.speed = 25
