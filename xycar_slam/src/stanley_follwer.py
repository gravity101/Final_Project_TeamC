#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
import time
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu,Image
import cv2
from cv_bridge import CvBridge

msg = xycar_motor()

class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.v = 0
        self.imu_data = -0.98
        self.now = time.time()
        self.prev_now = time.time()
        
        with open("/home/nvidia/xycar_ws/src/xycar_slam/final9.pkl", "rb") as f:
            self.path = pickle.load(f)
        self.rear_x_previous = self.path['x'][0]
        self.rear_y_previous = self.path['y'][0]

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)

    def ImuCallBack(self, msg):
        self.imu_data = msg.linear_acceleration.z

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        dx = self.rear_x - self.rear_x_previous
        dy = self.rear_y - self.rear_y_previous
        
        self.now = time.time()
        
        dt = self.now - self.prev_now
        dist = np.hypot(dx, dy)
        self.v = float(dist) / float(dt)
        self.rear_x_previous = self.rear_x
        self.rear_y_previous = self.rear_y
        
        self.prev_now = self.now
        

    def GetMsg(self):
      delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'], self.path['y'], self.path['yaw'],
                               0.3, 0.2)
      
      if delta <-50:
        delta=-50
      elif delta >50:
        delta = 50
        
      
      msg.angle=delta
      msg.speed=25
      return msg

if __name__ == '__main__':
    rospy.init_node("stanley_follower_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    stanley = StanleyController()
    msg.speed = 15
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        x = stanley.rear_x
        y = stanley.rear_y
        msg = stanley.GetMsg()
        motor_pub.publish(msg)


        r.sleep()
