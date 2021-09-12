#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
ar_parking_flag = True
ar_drive_flag = False
temp = 0
angle = 0
speed = 0
final_drive_flag = True
atan =0
lidar_points=None
flag1 = False
flag2 = False
flag3 = True

def ar_callback(msg):
    global arData
    global ar_parking_flag, ar_drive_flag, temp, atan
    
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        
        
        temp = i.id
        if(temp == 0):
          ar_parking_flag = False
          #print("parking start")
        
        if (temp == 9) :
          ar_drive_flag = True
          #print("check")
    atan = math.degrees(math.atan2(arData["DX"], arData["DY"]))



def lidar_callback(data):
  global lidar_points
  lidar_points = data.ranges

rospy.init_node('ar_parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
xycar_msg = xycar_motor()
while not rospy.is_shutdown():
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"], arData["AY"],
                                            arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    """
    print("=======================")
    print(" roll  : " + str(round(roll,1)))
    print(" pitch : " + str(round(pitch,1)))
    print(" yaw   : " + str(round(yaw,1)))

    print(" x : " + str(round(arData["DX"],4)))
    print(" y : " + str(round(arData["DY"],4)))
    print(" z : " + str(round(arData["DZ"],4)))
    print("angle : " + str(round(xycar_msg.angle,0)))
    print("speed : " + str(round(xycar_msg.speed,0)))
    print("id : " + str(temp))
    print("atan : " + str(atan))
    """
    
    if lidar_points == None:
        print("none")
        continue
    else:
      #for j in range(1):
        #print(lidar_points[j])    
        #rospy.sleep(0.1)
      #print("-------------------------")
    
      count1 =0
      for i in range(0,21):      
        if (0.7 <= lidar_points[i] <= 0.74):
          count1 +=1
          #print("find: ",i)
          if (count1 >= 5):
            flag1 =True
            print("flag1")
      
      count =0 
      for i in range(30,51):      
        if (1.54 <= lidar_points[i] <= 1.62):
          count +=1
          #print("find: ",i)
          if (count >= 5):
            flag2 =True
            print("flag2")      
      if(flag1 and flag2):
        
        xycar_msg.angle = 0
        xycar_msg.speed = 20
        motor_pub.publish(xycar_msg)
        time.sleep(0.5)
        
        xycar_msg.angle = 0
        xycar_msg.speed = 0
        motor_pub.publish(xycar_msg)
        time.sleep(2.5)
      
        for j in range(50):
          xycar_msg.angle = -50
          xycar_msg.speed = -30
          motor_pub.publish(xycar_msg)
          if arData["DZ"] > 0.7:
            print("enough")
            break
          time.sleep(0.5)
          
        
        if arData["DZ"] > 0.7:
          for c in range(5):
            xycar_msg.angle = 0
            xycar_msg.speed = 0
            motor_pub.publish(xycar_msg)
            flag3 = False
            print("flag3 is False")
            time.sleep(0.5)  
      
      if(flag3 == True):
          xycar_msg.angle = 0
          xycar_msg.speed = 25
          motor_pub.publish(xycar_msg)
          #time.sleep(0.5)
      

    """
    if ar_drive_flag:
      if not ar_parking_flag:
        ar_drive_flag = False
      left_drive(-50, 50)
    """
    #parking_dirve()
      
cv2.destrouAllWindows()
