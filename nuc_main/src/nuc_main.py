#!/usr/bin/env python

import rospy
import os
import time

import handlers
import drive
import config

if __name__ == '__main__':
    print("nuc_main started")

    drive.init()
    handlers.init_handlers()

    axis_log_time = -10000

    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(),
                   rospy.core.get_node_uri(), os.getpid())
    try:
        while not rospy.core.is_shutdown():
            if time.time() > axis_log_time + config.AXIS_LOG["PERIOD"]:
                try:
                    states = drive.get_states()
                    print(states)
                except Exception as e:
                    handlers.debug_publish("ERROR in get_states: " + str(e))
                axis_log_time = time.time()
                # handlers.axis_states_publish(states)
            rospy.rostime.wallsleep(0.001)
    except KeyboardInterrupt:
        rospy.logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
