#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import OccupancyGrid

def talker():
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    rospy.init_node('map_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        msg = OccupancyGrid()
        msg.info.width = 100
        msg.info.height = 100
        msg.data = [0, 0, 0]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
