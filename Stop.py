#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)
stopline_threshold = 125
area_threshold = 2000
lengh_threshold = 300
check = False


def img_callback(data) :
    global image, check
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    check = process_image(image)
    
def process_image(data):
        blur = cv2.GaussianBlur(data, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, lane = cv2.threshold(L, stopline_threshold, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        detected = False
        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)
            if not ((area > area_threshold) and (length > lengh_threshold)):
                continue
            if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
                continue
            x, y, w, h = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            _, width, _ = data.shape
            if (200 <= center[0] <= (width - 200)) and (w > 400) & (h < 80):
                cv2.rectangle(data, (x, y), (x + w, y + h), green, 2)
                detected = True
        cv2.imshow('stopline', data)
        return detected

rospy.init_node('check')

image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
msg = xycar_motor()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        msg.speed = 10
        msg.angle = 0
        if check == True :
            msg.speed = 0
        pub.publish(msg)
        
        rospy.sleep(1.0)
