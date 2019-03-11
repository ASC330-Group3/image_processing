#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('talker')
rate = rospy.Rate(10) # 10hz

msg = Twist()

while not rospy.is_shutdown():
    #Add loop in here

    msg.linear.x = 1
    msg.linear.y = 0#0.25
    msg.angular.z = 0

    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
