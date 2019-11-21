#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np


angles_target = np.zeros(3)

def arm_angles_target_callback(data):
    global angles_target
    angles_target = np.array([data.x, data.y, data.z])


pub_arm_angles = rospy.Publisher('arm_angles', Vector3, queue_size=10)

speeds = np.ones(3)*np.pi
angles = np.zeros(3)

rospy.Subscriber('arm_angles_target', Vector3, arm_angles_target_callback)
rospy.init_node('arm_model')

PUB_RATE = 10.0

rate = rospy.Rate(PUB_RATE)
while not rospy.is_shutdown():
    """
    angles_dif = angles_target - angles
    angles_delta = speeds * (1/PUB_RATE) * np.sign(angles_dif)
    for i in range(angles.size):
        if abs(angles_dif[i]) <= abs(angles_delta[i]):
            angles[i] = angles_target[i]
        else:
            angles[i] += angles_delta[i]
    """
    angles = angles_target # For now, just remove any delay
    arm_angles_out = Vector3(*angles)
    pub_arm_angles.publish(arm_angles_out)
    rate.sleep()