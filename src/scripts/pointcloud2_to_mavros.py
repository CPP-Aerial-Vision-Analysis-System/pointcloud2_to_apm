#! /usr/bin/env python

import rospy
from mavros_msgs.msg import Mavlink
from mavros import mavlink as mavlink_ros
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import time

mavlink_instance = mavlink2.MAVLink(None)

start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
current_time_ms = current_milli_time()

def main():
    rospy.init_node("offb_node_py")

    rate = rospy.Rate(10)

    pub = rospy.Publisher("/mavlink/to", data_class = Mavlink, queue_size = 20)

    while not rospy.is_shutdown():

        # create the OBSTACLE_DISTANCE_3D data structure
        obstacle_msg = mavlink2.MAVLink_obstacle_distance_3d_message(
            time_boot_ms = round(time.time() * 1000) - start_time,
            sensor_type= 0,
            frame = mavlink2.MAV_FRAME_BODY_FRD,
            obstacle_id = 65535, # max uint16_t
            x = 1,
            y = 0,
            z = 0,
            min_distance = .2,
            max_distance = 25
        )

        # populate internal headers
        obstacle_msg.pack(mavlink_instance)

        # convert to ROS message
        rosmsg = mavlink_ros.convert_to_rosmsg(obstacle_msg)

        # manually update mavlink version
        rosmsg.magic = mavlink2.PROTOCOL_MARKER_V2

        pub.publish(rosmsg)

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass