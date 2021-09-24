#!/usr/env/bin python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import time

from final_msg.msg import BumpMsg, InterruptMsg, ObstacleMsg, ParkingMsg, StopLineMsg, TParkingMsg, TrackingMsg, TrafficLightMsg, YoloMsg
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor

motor = xycar_motor()
tracking_data = None


def callback_tracking(msg):
    global tracking_data
    tracking_data = [msg.status, msg.angle, msg.speed]

    
def GetMotor():
    global tracking_data
    
    if tracking_data[0] == True:
        motor.angle = tracking_data[1]
        motor.speed = tracking_data[2]


if __name__ == '__main__':
    rospy.init_node("final_main_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    rospy.Subscriber("bump_topic", BumpMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    rospy.Subscriber("tracking_topic", TrackingMsg, callback_tracking)
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        motor = GetMotor()
        motor_pub.publish(motor)

        r.sleep()

