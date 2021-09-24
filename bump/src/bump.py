#!/usr/bin/env python
import rospy
import time
import numpy as np
import time
import cv2

from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu,Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge

motor = xycar_motor()

class Bumper(object):
    def __init__(self):
        self.yaw = 0.0
        self.pitch=0.0
        self.roll=0.0
        self.imu_data = None

        self.imu_sub = rospy.Subscriber("imu", Imu, self.callback_imu)

    def callback_imu(self, msg):        
        self.imu_data = [msg.orientation.x, msg.orientation.y, data.orientation.z, data.orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])

        print('Roll:%.4f, Pitch:%.4f, Yaw:%.4f' % (self.roll, self.pitch, self.yaw))

    def GetMotor(self):

      angle = 0
      speed = 0
      print("GetMotor Function Working")
        
      motor.angle = angle
      motor.speed = speed
      return motor
    

if __name__ == '__main__':
    rospy.init_node("bump_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    bump = Bumper()
    r = rospy.Rate(100)

    while not rospy.is_shutdown():

        if bump.imu_data == None:
            continue

        motor = bump.GetMotor()
        motor_pub.publish(motor)


        r.sleep()

