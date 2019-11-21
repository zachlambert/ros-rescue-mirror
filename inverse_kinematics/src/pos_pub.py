#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np

from kinematics import angles_to_cartesian, angles_to_cylindrical

pub_cartesian = rospy.Publisher('arm_cartesian', Vector3, queue_size=10)
pub_cylindrical = rospy.Publisher('arm_cylindrical', Vector3, queue_size=10)

def arm_angles_callback(vector_in):
    angles = np.array([vector_in.x, vector_in.y, vector_in.z])
    cartesian = angles_to_cartesian(angles)
    cylindrical = angles_to_cylindrical(angles)
    pub_cartesian.publish(Vector3(*cartesian))
    pub_cylindrical.publish(Vector3(*cylindrical))

rospy.Subscriber('arm_angles', Vector3, arm_angles_callback)
rospy.init_node('pos_pub')
rospy.spin()