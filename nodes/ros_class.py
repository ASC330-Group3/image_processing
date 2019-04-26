#!/usr/bin/env python
"""
Created on Thur Apr 26

@author: Marwan Taher
"""

import sys
sys.path.insert(0, '/home/mk_99/catkin_ws/src/platform_nav/OpenCV')

import tf
import rospy
import CostMap as cm
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from PyQt5.QtCore import QThread, pyqtSignal

class comms(QThread):

    map_msg = 0
    transform_data = {"state" : 0}

    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        # Initializing Node
        rospy.init_node('custom_script', anonymous=True)
        rospy.loginfo("ROS thread initializing...")

        #rospy.Subscriber("move_base/status", GoalStatusArray, self.nav_state_callback)
        #rospy.Subscriber("machining_station_state", String, self.machining_station_state_callback)

        # Keeps python from exiting until this node is stopped
        # rospy.spin()

    def nav_state_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Navigation state message reactived:\n %s", data.data)
        self.nav_state = data.data

    def machining_station_state_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Navigation state message reactived:\n %s", data.data)
        self.machining_station_state = data.data


    def run(self):
        rospy.loginfo("ROS thread started.")
        self.running = True

        tf_pub = tf.TransformBroadcaster()
        #rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
        map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        nav_goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

        rate = rospy.Rate(20) # 10hz

        old_x = 0
        old_y = 0
        old_th = 0
        last_time = rospy.Time.now()

        global_frame = "map" # odom
        child_frame = "base_link"

        self.map_msg = OccupancyGrid()
        self.map_msg.info.width = 640  # Map width
        self.map_msg.info.height = 480  # Map height
        self.map_msg.info.resolution = 0.005 # Map resolution
        self.map_msg.header.frame_id = global_frame # New

        odom_msg = Odometry()
        odom_msg.header.frame_id = global_frame # odom
        odom_msg.child_frame_id = child_frame

        nav_goal_msg = PoseStamped()
        nav_goal_msg.header.frame_id = global_frame
        self.set_nav_goal = {"state" : False}

        while self.running and not rospy.is_shutdown():
            #rospy.loginfo(msg)

            '''***************************************
            ** Needs to be passed in the class
            **transform_data = raw_map.get_transform()
            ***************************************'''

            current_time = rospy.Time.now()

            self.map_msg.header.stamp = current_time # New
            self.map_msg.info.map_load_time= current_time # New

            '''***************************************
            **Needs to be passed in the class
            **map_msg.data = raw_map.get_new_frame() #pass Frame here
            ***************************************'''

            # Publishing the map
            map_pub.publish(self.map_msg)

            try:
                if self.transform_data["state"] == 1:
                    #rospy.loginfo("New tf sent!")

                    # Getting the robot's position
                    x = self.transform_data["x"]*0.005
                    y = self.transform_data["y"]*0.005
                    th = self.transform_data["angle"]
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
                    '''
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
                    '''
            except Exception as e:
                rospy.loginfo("Was about to die, check error below:")
                rospy.loginfo(e)

            if self.set_nav_goal["state"]:
                nav_goal_msg.header.stamp = rospy.Time.now()

                nav_goal_msg.pose.pose.position.x = self.set_nav_goal["x"]
                nav_goal_msg.pose.pose.position.y = self.set_nav_goal["y"]

                nav_goal_odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.set_nav_goal["th"])
                nav_goal_msg.pose.pose.orientation = Quaternion(*nav_goal_odom_quat)

                nav_goal_pub.publish(nav_goal_msg)
                
            #rospy.loginfo("In the thread!!")
            rate.sleep()

        rospy.loginfo("ROS thread stopped")

    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == "__main__":
    raw_map = cm.map_capture(1)

    ros_comms = comms()
    ros_comms.start()

    while not rospy.is_shutdown():
        ros_comms.transform_data = raw_map.get_transform()
        ros_comms.map_msg.data = raw_map.get_new_frame()
        #rospy.loginfo("In the looop")
