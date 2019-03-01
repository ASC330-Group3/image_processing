#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import OccupancyGrid


pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
rospy.init_node('map_publisher', anonymous=True)
rate = rospy.Rate(10) # 10hz


#start init for opencv

#end init for opencv


while not rospy.is_shutdown():
    #Add loop in here

    msg = OccupancyGrid()
    msg.info.width = 100  #resolution width
    msg.info.height = 100 #resolution height
    msg.data = [0, 0, 0] #pass Frame here

    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
