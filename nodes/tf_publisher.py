#!/usr/bin/env python
#Source: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

import roslib
roslib.load_manifest('platform_nav')

import rospy
import tf

rospy.init_node('tf_broadcaster')
#platform_name = rospy.get_param('~platform') # Reutrns the argument from launch file
rate = rospy.Rate(20) # 10hz
br = tf.TransformBroadcaster()


pose ={ # For testing
    "x":3,
    "y":1,
    "theta":3.14
}

while not rospy.is_shutdown():
    br.sendTransform((pose["x"], pose["y"], 0),
                     tf.transformations.quaternion_from_euler(0, 0, pose["theta"]),
                     rospy.Time.now(),
                     "/base_link", # platform_name
                     "/map")
    rate.sleep()
