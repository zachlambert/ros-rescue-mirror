#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('/pingpong', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(2)  # 10hz
while not rospy.is_shutdown():
    ping_str = "ping %s" % rospy.get_time()
    pub.publish(ping_str)
    rate.sleep()
