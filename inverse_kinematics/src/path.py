#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np

from kinematics import path_cartesian_to_angles, path_cylindrical_to_angles


cartesian_pos = None
cylindrical_pos = None
path = []
path_i = 0
pub_pos = None
PATH_PUB_RATE = 10 #Hz


def arm_cartesian_callback(vector_in):
    global cartesian_pos
    cartesian_pos = np.array([vector_in.x, vector_in.y, vector_in.z])
    

def arm_cylindrical_callback(vector_in):
    global cylindrical_pos
    cylindrical_pos = np.array([vector_in.x, vector_in.y, vector_in.z])


def publish_path_angles(pos, vector_in, conversion_func):
    global path, path_i
    target_pos = np.array([vector_in.x, vector_in.y, vector_in.z])
    speed = 10 # Put this in the arm_target message later
    distance = np.linalg.norm(target_pos - pos)
    time = distance/speed
    num_points = int(np.floor(time*PATH_PUB_RATE)) + 1
    angles = conversion_func(pos, target_pos, num_points)
    path = angles[1:] #Don't need current_pos in path
    path_i = 0


def arm_cartesian_target_callback(vector_in):
    if cartesian_pos is None:
        return
    publish_path_angles(cartesian_pos, vector_in, path_cartesian_to_angles)


def arm_cylindrical_target_callback(vector_in):
    if cylindrical_pos is None:
        return
    publish_path_angles(cylindrical_pos, vector_in, path_cylindrical_to_angles)


pub_angles_target = rospy.Publisher('arm_angles_target', Vector3, queue_size=10)

rospy.Subscriber('arm_cartesian', Vector3, arm_cartesian_callback)
rospy.Subscriber('arm_cylindrical', Vector3, arm_cylindrical_callback)

rospy.Subscriber('arm_cartesian_target', Vector3, arm_cartesian_target_callback)
rospy.Subscriber('arm_cylindrical_target', Vector3, arm_cylindrical_target_callback)

rospy.init_node('path')
rate = rospy.Rate(PATH_PUB_RATE)

while not rospy.is_shutdown():
    if len(path)!=0 and path_i<len(path):
        angles = path[path_i]
        path_i+=1
        vector_out = Vector3(*angles)
        pub_angles_target.publish(vector_out)
    rate.sleep()