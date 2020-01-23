#!/usr/bin/env python

import rospy

import handlers
import state_publisher

if __name__=="__main__":
    rospy.init_node('state_publisher')
    handlers.init()
    state_publisher.run()
