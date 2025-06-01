#!/usr/bin/env python3

import struct
import rospy
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math
from pymavlink import mavutil
import time

rospy.init_node('send_obstacle_3D', anonymous=True)

time.sleep(1)

pub = rospy.Publisher('/send_obstacle_3D', PointCloud2, queue_size=10)

import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

master= mavutil.mavlink_connection('tcp:localhost:5762', dialect='ardupilotmega')

start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
current_time_ms = current_milli_time()

done = True

rate = rospy.Rate(.1)

def lidar_callback(data):
    """
    Callback function for processing PointCloud2 data and publishing MAVLink messages.
    """

    global pub, rate, done

    # print("got callback")

    if(not done):
        return

    # Convert PointCloud2 to a list of points
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    # <19.7499
    top = []
    # 19.7499 - 53.966
    mid = [[]for i in range(6)]
    # >53.966
    bot = [[] for i in range(10)]

    cloud_itr = 0
    
    for point in points:
        x = point[0]
        y = point[1]
        z = point[2]
        if x == 0 and y == 0 or x == 0 and y == 0 and z == 0:
            cloud_itr += 1
            continue

        r = math.sqrt(x**2 + y**2 + z**2)
        theta = math.atan2(y, x) * 180 / math.pi
        if theta < 0:
            theta = theta + 360
        phi = math.atan2(math.sqrt(x * x + y * y), z) * 180 / math.pi
        if phi < 0:
            phi = phi + 360

        # if r < 1:

        if phi <= 19.7499:
            top.append((r, theta, phi, cloud_itr))
        elif phi > 19.7499 and phi <= 53.966:
            mid[int(theta * 6 / 360)].append((r, theta, phi, cloud_itr))
        else:
            bot[int(theta * 10 / 360)].append((r, theta, phi, cloud_itr))
    
        cloud_itr += 1
    
    def get_min(p1):
        if(len(p1) > 0):
            lowest = 1000
            lowest_itr = None

            for i in p1:
                if i[0] < lowest:
                    lowest = i[0]
                    lowest_itr = i[3]

            return(lowest_itr)
        
        else: return(None)

    finalList = []

    
    top_min = get_min(top)
    if top_min is not None:
        finalList.append(points[top_min])
    for i in mid:
        min = get_min(i)
        if min is not None:
            finalList.append(points[min])   
    for i in bot :
        min = get_min(i)
        if min is not None:
            finalList.append(points[min])   

    seg = []

    for i in bot[9]:
        seg.append(points[i[3]])
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'unilidar_lidar'

    scaled_polygon_pcl = pc2.create_cloud_xyz32(header, finalList)
    # scaled_polygon_pcl = pc2.create_cloud_xyz32(header, seg)

    # Publish the messages
    pub.publish(scaled_polygon_pcl)
    
    num = 0

    for point in finalList:
        obstacle_x = point[0]
        obstacle_y = point[1]
        obstacle_z = point[2]

        print(num, point)

        # Sensor and frame configuration
        sensor_type = 0  # Laser
        obstacle_id = 1
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        # Create the raw MAVLink message
        raw_msg = master.mav.obstacle_distance_3d_send(
            time_boot_ms = current_time_ms * 1000,   # Current time in microseconds
            sensor_type = 0,
            frame= mavutil.mavlink.MAV_FRAME_BODY_FRD,
            obstacle_id= 65535,
            x=float(obstacle_x),
            y=float(-obstacle_y),
            z=float(-obstacle_z),
            
            min_distance=float(.35),
            max_distance=float(10)
        )

        num += 1
    
    print()

def main():
    
    # Initialize the ROS node
    
    # Subscriber for unitree
    # rospy.Subscriber('/ScanCombine', PointCloud2, lidar_callback)
    rospy.Subscriber('/unilidar/cloud', PointCloud2, lidar_callback)
    # rospy.loginfo("UniLidar subscriber and MAVLink publisher node started.")
    rate.sleep()
    rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass