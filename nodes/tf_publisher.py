#!/usr/bin/env python
#Source: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

import roslib
import rospy
import tf

rospy.init_node('platform_tf_broadcaster')
platform_name = rospy.get_param('~platform') # Reutrns the argument from launch file
rate = rospy.Rate(10) # 10hz
br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    br.sendTransform((pose.x, pose.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, pose.theta),
                     rospy.Time.now(),
                     "base_link", # platform_name
                     "world")
    rate.sleep()
