#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np

from kinematics import cartesian_to_angles, cylindrical_to_angles


pub_angles_target = rospy.Publisher('arm_angles_target', Vector3, queue_size=10)


def publish_angles(vector_in, conversion_func):
    pos = np.array([vector_in.x, vector_in.y, vector_in.z])
    angles = conversion_func(pos)
    vector_out = Vector3(*angles)
    pub_angles_target.publish(vector_out)


def arm_cartesian_target_callback(vector_in):
    publish_angles(vector_in, cartesian_to_angles)


def arm_cylindrical_target_callback(vector_in):
    publish_angles(vector_in, cylindrical_to_angles)


rospy.Subscriber('arm_cartesian_target', Vector3, arm_cartesian_target_callback)
rospy.Subscriber('arm_cylindrical_target', Vector3, arm_cylindrical_target_callback)
rospy.init_node('single')
rospy.spin()