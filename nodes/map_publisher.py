#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import OccupancyGrid
import tf
import sys
sys.path.insert(0, '/home/mk_99/catkin_ws/src/platform_nav/OpenCV')

import CostMap as cm

pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
rospy.init_node('map_publisher', anonymous=True)

br = tf.TransformBroadcaster()

rate = rospy.Rate(10) # 10hz

#start init for opencv
map = cm.map_capture(1)
#end init for opencv

while not rospy.is_shutdown():
    #Add loop in here
    msg = OccupancyGrid()
    msg.info.resolution = 0.005
    msg.info.width = 640  #resolution width
    msg.info.height = 480 #resolution height
    msg.data = map.get_new_frame() #pass Frame here

    #rospy.loginfo(msg)
    pub.publish(msg)


    transform_data = map.get_transform()

    if transform_data["state"] == 1:
        br.sendTransform((transform_data["x"]*0.005, transform_data["y"]*0.005, 0),
                         tf.transformations.quaternion_from_euler(0, 0, transform_data["angle"]),
                         rospy.Time.now(),
                         "/base_link", # platform_name
                         "/map")
    else:
        rospy.loginfo("tf failed")

    rate.sleep()
