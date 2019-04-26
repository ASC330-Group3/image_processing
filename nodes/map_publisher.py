#!/usr/bin/env python
"""
Created on Wed Feb 15

@author: Marwan Taher
"""
import sys
sys.path.insert(0, '/home/mk_99/catkin_ws/src/platform_nav/OpenCV')

import tf
import rospy
import CostMap as cm
from nav_msgs.msg import Odometry, OccupancyGrid
#from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

raw_map = cm.map_capture(1)

tf_pub = tf.TransformBroadcaster()
#rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

# Initializing Node
rospy.init_node('custom_script', anonymous=True)

rate = rospy.Rate(20) # 10hz

old_x = 0
old_y = 0
old_th = 0
last_time = rospy.Time.now()

global_frame = "map" # odom
child_frame = "base_link"

map_msg = OccupancyGrid()
map_msg.info.width = 640  # Map width
map_msg.info.height = 480  # Map height
map_msg.info.resolution = 0.005 # Map resolution
map_msg.header.frame_id = global_frame # New

odom_msg = Odometry()
odom_msg.header.frame_id = global_frame # odom
odom_msg.child_frame_id = child_frame

while not rospy.is_shutdown():
    #rospy.loginfo(msg)
    transform_data = raw_map.get_transform()

    current_time = rospy.Time.now()

    map_msg.header.stamp = current_time # New
    map_msg.info.map_load_time= current_time # New
    map_msg.data = raw_map.get_new_frame() #pass Frame here

    # Publishing the map
    map_pub.publish(map_msg)

    try:
        if transform_data["state"] == 1:
            #rospy.loginfo("New tf sent!")

            # Getting the robot's position
            x = transform_data["x"]*0.005
            y = transform_data["y"]*0.005
            th = transform_data["angle"]
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # Publshing the transforms
            tf_pub.sendTransform((x, y, 0),
                             odom_quat,
                             current_time,
                             child_frame, # child frame
                             global_frame) # Parent frame odom

            odom_msg.header.stamp = current_time

            # Setting the pose of the robot
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0
            odom_msg.pose.pose.orientation = Quaternion(*odom_quat)

            # calculating velocities
            dt = (current_time - last_time).to_sec()
            vx = (x - old_x)/dt
            vy = (y - old_y)/ dt
            vth = (th - old_th)/ dt

            # Setting the velocities
            odom_msg.twist.twist.linear = Vector3(vx, vy, 0)
            odom_msg.twist.twist.angular = Vector3(0, 0, vth)

            # Publishing the odometery message
            odom_pub.publish(odom_msg)

            # Recording the values for the next loop Iteration
            old_x = x
            old_y = y
            old_th = th
            last_time = current_time

        else:
            #rospy.loginfo("lol tf failed")

            # Getting the robot's position
            x = 1
            y = 1
            th = 0
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # Publshing the transforms
            tf_pub.sendTransform((x, y, 0),
                             odom_quat,
                             current_time,
                             child_frame, # child frame
                             global_frame) # Parent frame odom

            odom_msg.header.stamp = current_time

            # Setting the pose of the robot
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0
            odom_msg.pose.pose.orientation = Quaternion(*odom_quat)

            # calculating velocities
            dt = (current_time - last_time).to_sec()
            vx = (x - old_x)/dt
            vy = (y - old_y)/ dt
            vth = (th - old_th)/ dt

            # Setting the velocities
            odom_msg.twist.twist.linear = Vector3(vx, vy, 0)
            odom_msg.twist.twist.angular = Vector3(0, 0, vth)

            # Publishing the odometery message
            odom_pub.publish(odom_msg)

    except:
        #rospy.loginfo(str(transform_data))
        #rospy.loginfo("Was about to die")
        pass
    rate.sleep()
