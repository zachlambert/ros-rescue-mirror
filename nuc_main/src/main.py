#!/usr/bin/env python

import rospy
import os
import time

import handlers
import drive
import config

from dynamic_reconfigure.server import Server
from nuc_main.cfg import OdriveConfig

def config_callback(config, level):
    rospy.loginfo("Reconfigure request: " + repr(config))
    return config

if __name__ == '__main__':
    print("nuc_main started")
    
    handlers.init_handlers()
    
    srv = Server(OdriveConfig, config_callback)
    
    drive.init()

    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    
    rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
    r = rospy.Rate(10)
    try:
        while not rospy.core.is_shutdown():
            states = drive.get_states()
            print(states)
            r.sleep()
    except KeyboardInterrupt:
        rospy.logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
