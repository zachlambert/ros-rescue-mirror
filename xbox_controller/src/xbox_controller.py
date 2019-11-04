#!/usr/bin/env python

import rospy
import os
import time
import inputs
import threading

import handlers
from config import *


controls = {}


def gamepad_handler():
    global x, y
    while True:
        try:
            events = inputs.get_gamepad()
            for event in events:
                # print(event.ev_type, event.code, event.state)
                if event.code in ("ABS_X", "ABS_Y"):
                    mx = max(abs(controls.get("ABS_X", 0)), abs(controls.get("ABS_Y", 0)), abs(int(event.state)))
                    x = int(event.state) if mx > 8000 else 0
                elif event.code in ("ABS_RX", "ABS_RY"):
                    mx = max(abs(controls.get("ABS_RX", 0)), abs(controls.get("ABS_RY", 0)), abs(int(event.state)))
                    x = int(event.state) if mx > 8000 else 0
                else:
                    x = int(event.state)
                controls[event.code] = x
                if event.code in TRIGGERS:
                    topic, message = MESSAGES[TRIGGERS[event.code]](controls)
                    handlers.publish_msg(topic, message)
        except Exception as e:
            print(e)
            time.sleep(0.1)


if __name__ == '__main__':

    handler_thread = threading.Thread(target=gamepad_handler)
    handler_thread.start()

    handlers.init_handlers()

    axis_log_time = -10000

    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(),
                   rospy.core.get_node_uri(), os.getpid())

    try:
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.001)

    except KeyboardInterrupt:
        rospy.logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
