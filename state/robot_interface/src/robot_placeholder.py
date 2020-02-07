#!/usr/bin/env python

import copy

import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('robot_placeholder')

arm_pub = rospy.Publisher('/arm_current_angles', Float32MultiArray, queue_size=10)
wrist_pub = rospy.Publisher('/wrist_current_angles', Float32MultiArray, queue_size=10)

def arm_callback(msg):
    arm_pub.publish(msg)

def wrist_callback(msg):
    wrist_pub.publish(msg)

arm_sub = rospy.Subscriber('/arm_demand_angles', Float32MultiArray, arm_callback)
wrist_sub = rospy.Subscriber('/wrist_demand_angles', Float32MultiArray, wrist_callback)

rospy.spin()
