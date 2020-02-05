#!/usr/bin/env python

import rospy

import handlers
import state_publisher
import info_publisher

if __name__=="__main__":
    rospy.init_node('state_publisher')
    handlers.init()
    state_publisher.init()
    info_publisher.init()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        state_publisher.publish()
        info_publisher.publish()

        rate.sleep()
