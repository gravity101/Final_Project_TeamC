#!/usr/bin/env python

import rospy
from xycar_msgs.msg import xycar_motor

msg = xycar_motor()

if __name__ == '__main__':
    rospy.init_node("curvature_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    msg.speed = 30
    msg.angle = -50

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        motor_pub.publish(msg)

        r.sleep()
