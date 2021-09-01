#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

name = "xycar_position_check"
sub_topic = "tracked_pose"

xycar_pose = [0, 0, 0, 0, 0, 0]

def callback(data):
    global xycar_pose

    roll, pitch, yaw = euler_from_quaternion (
        [
            data.pose.orientation.x, \
            data.pose.orientation.y, \
            data.pose.orientation.z, \
            data.pose.orientation.w \
        ]
    )

    xycar_pose[0] = data.pose.position.x
    xycar_pose[1] = data.pose.position.y
    xycar_pose[2] = data.pose.position.z
    xycar_pose[3] = roll
    xycar_pose[4] = pitch
    xycar_pose[5] = yaw

pub = rospy.Publisher('path_draw', MarkerArray)
markerArray = MarkerArray()
count = 0
MARKERS_MAX = 100

rospy.init_node(name, anonymous=True)
rospy.Subscriber(sub_topic, PoseStamped, callback)


rate = rospy.Rate(10)

while not rospy.is_shutdown():
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = xycar_pose[0]
    marker.pose.position.y = xycar_pose[1]
    marker.pose.position.z = xycar_pose[2]
 

    #if(count > MARKERS_MAX):
    #    markerArray.markers.pop(0)
 
    markerArray.markers.append(marker)
 
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
 
    pub.publish(markerArray)
    count += 1


    print("x : {}".format(xycar_pose[0]))
    print("y : {}".format(xycar_pose[1]))
    print("z : {}".format(xycar_pose[2]))
    print("roll : {}".format(xycar_pose[3]))
    print("pitch : {}".format(xycar_pose[4]))
    print("yaw : {}".format(xycar_pose[5]))
    print("-------------------------------------")
    rate.sleep()

