import numpy as np

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def StanleyControl(x, y, yaw, v, present_time, map_xs, map_ys, map_yaws, time_log, L, k):
    weight = 0.001
    # find nearest waypoint
    min_dist = 1e9
    min_index = 0
    n_points = len(map_xs)

    # convert rear_wheel x-y coordinate to front_wheel x-y coordinate
    # L = wheel base
    front_x = x - L * np.cos(yaw) 
    front_y = y - L * np.sin(yaw) 
    #print("present: ",present_time)
    for i in range(n_points):
        # calculating distance (map_xs, map_ys) - (front_x, front_y)
        dx = front_x - map_xs[i]
        dy = front_y - map_ys[i]
        dist = np.hypot(dx, dy)
        #print("time_log: ", time_log[i] - present_time)
        #print("time-present_time: ",time_log[i] - present_time )
        if dist < min_dist:
            min_dist = dist
            min_index = i
            print("min_index", min_index)

    # compute cte at front axle
    map_x = map_xs[min_index]
    map_y = map_ys[min_index]
    map_yaw = map_yaws[min_index]
    
    # nearest x-y coordinate (map_x, map_y) - front_wheel coordinate (front_x, front_y)
    dx = map_x - front_x
    dy = map_y - front_y
    #print("map_time: ", map_time)
    perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
    cte = np.dot([dx, dy], perp_vec)

    # control law
    # heading error = yaw_term
    yaw_term = normalize_angle(map_yaw - yaw)
    cte_term = np.arctan2(k*cte, v)
    print("CTE",cte_term*180/np.pi)
    print("yaw_term",-yaw_term*180/np.pi)
    #rospy.sleep(0.2)
    # steering
    #steer = weight*(-yaw_term)*180/np.pi + (1-weight)*(-cte_term)*180/np.pi
    steer = (-yaw_term)*180/np.pi + cte_term*180/np.pi
    return steer

