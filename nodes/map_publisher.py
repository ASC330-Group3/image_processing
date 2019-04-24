#!/usr/bin/env python
"""
Created on Wed Feb 15

@author: Marwan Taher
"""
import sys
sys.path.insert(0, '/home/mk_99/catkin_ws/src/platform_nav/OpenCV')

import tf
import rospy
import math
import CostMap as cm
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

map = cm.map_capture(1)

pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

rospy.init_node('custom_script', anonymous=True)

br = tf.TransformBroadcaster()

rate = rospy.Rate(10) # 10hz

old_x = 0
old_y = 0
old_th = 0
last_time = rospy.Time.now()

msg = OccupancyGrid()
msg.info.resolution = 0.005
msg.info.width = 640  #resolution width
msg.info.height = 480 #resolution height

while not rospy.is_shutdown():
    transform_data = map.get_transform()

    msg.data = map.get_new_frame() #pass Frame here
    pub.publish(msg)
    #rospy.loginfo(msg)

    if transform_data["state"] == 1:
        br.sendTransform((transform_data["x"]*0.005, transform_data["y"]*0.005, 0),
                         tf.transformations.quaternion_from_euler(0, 0, transform_data["angle"]),
                         rospy.Time.now(),
                         "base_link", # child frame
                         "odom") # Parent frame
        #rospy.loginfo("New tf sent!")

        current_time = rospy.Time.now()
        '''
        odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
        )
        '''

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        current_time = rospy.Time.now()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        x = transform_data["x"]*0.005
        y = transform_data["y"]*0.005
        th = transform_data["angle"]
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # set the position
        #odom.pose.pose = Pose(Point(str(x), str(y), "0"), str(Quaternion(odom_quat)))

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0

        odom.pose.pose.orientation = Quaternion(*odom_quat)

        dt = (current_time - last_time).to_sec()
        vx = (x - old_x)/dt
        vy = (y - old_y)/ dt
        vth = (th - old_th)/ dt

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear = Vector3(vx, vy, 0)
        odom.twist.twist.angular = Vector3(0, 0, vth)

        # publish the message
        odom_pub.publish(odom)

        old_x = x
        old_y = y
        old_th = th
        last_time = current_time
    else:
        rospy.loginfo("lol tf failed")

    rate.sleep()
