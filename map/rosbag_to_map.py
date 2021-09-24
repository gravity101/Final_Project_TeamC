import rosbag
import pickle
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

init_yaw = 0.0
flag = True
temp = 0

if __name__ == '__main__':
    global init_yaw, flag, temp
    bag = rosbag.Bag("/home/nvidia/final_0917_7.bag")
    path = {'x': [], 'y': [], 'yaw': [], 'stamp': []}
    i = 0
    for topic, msg, t in bag.read_messages(topics=['/tracked_pose']):
        i += 1
        if i % 3 == 0:
            #continue
            print(i/3)

            orientation_q = msg.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            #path['x'].append(msg.pose.position.x + 0.1 * np.cos(yaw))
            #path['y'].append(msg.pose.position.y + 0.1 * np.sin(yaw))
            
            path['x'].append(msg.pose.position.x)
            path['y'].append(msg.pose.position.y)
            """
            if i < 30000:
                path['x'].append(msg.pose.position.x+ 0.12 * np.cos(yaw))
                path['y'].append(msg.pose.position.y+ 0.12 * np.sin(yaw))
            else:
                path['x'].append(msg.pose.position.x)
                path['y'].append(msg.pose.position.y)
            """
            #path['yaw'].append(yaw)
            path['stamp'].append(msg.header.stamp)
        #path['TIME'].append(int(str(msg.header.stamp))%1630640000000000000/1000)
        #print(int(str(msg.header.stamp))%1630640000000000000/1000)
    for topic, msg in bag.read_messages(topics=['/imu']):
        #j += 1
        #if j % 3 == 0:
        orientation_q2 = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([
        orientation_q2.x, orientation_q2.y, orientation_q2.z, orientation_q2.w])
          
        if (flag):
          init_yaw = yaw
          flag = False
        
        
        if(msg.header.stamp == path['stamp'][temp]):  
          path['yaw'].append(yaw - init_yaw)
          temp +=1
          
      
        
        
        
        
        
                    
    with open("final_0916_10.pkl", "wb") as f:
        pickle.dump(path, f)
