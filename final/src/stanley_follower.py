#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Imu

imu_flag = False
num = 0
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
        self.yaw_init =0.0
        self.yaw_term =0.0
        self.line_cte = 0.0
        
        
        with open("/home/nvidia/xycar_ws/src/map/final_0916_10.pkl", "rb") as f:
            self.path = pickle.load(f)
        
        self.tracked_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)
        self.line_cte_sub = rospy.Subscriber("line_cte", Int32, self.CTECallBack)
        
    def CTECallBack(self, msg):
        self.line_cte = msg.data

    def ImuCallBack(self, msg):
        global imu_flag
          
        self.imu_data = msg.linear_acceleration.z
        
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        if(not imu_flag):
          self.yaw_init = self.yaw
          
          imu_flag = True
        
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
      	#self.yaw +=1.0
        #print("yaw: ", self.yaw)
        self.yaw -= self.yaw_init
        #print("yaw: ", self.yaw)
        delta, idx, self.yaw_term = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v, self.index, 
                               self.path['x'], self.path['y'], self.path['yaw'], 0.07, 25)

        self.index = idx
        print(self.index)
        if delta < -21.155:
            delta = -21.155
        elif delta > 21.155:
            delta = 21.155

        angle = int(delta * 50 / 21.1549)
      
        if angle < -50:
            angle = -50
        elif angle > 50:
            angle = 50

        """
        if(abs(self.line_cte) > 150):
            print("line!!!!!!!!!!!!!!!!!!!!!!!")
            if (self.line_cte < 0):
              angle += 20
            else:
              angle -= 20
        """

        self.angle = angle
        self.speed = 15
        if (2800 < self.index < 3100) or (7900 < self.index < 9050):
            self.speed = 13
