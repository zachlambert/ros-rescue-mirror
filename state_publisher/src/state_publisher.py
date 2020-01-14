#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

import numpy as np

rospy.init_node('robot_tf_broadcaster')
joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
broadcaster = tf.TransformBroadcaster()
rate = rospy.Rate(10)

joint_state = JointState()

joint_names = ['base_link_to_flipper_front_right',
               'base_link_to_arm_lower',
               'arm_lower_to_arm_upper']
joint_angles = [0, 0, 0]

robot_x = 0
robot_y = 0
robot_angle = 0

while not rospy.is_shutdown():
    # Update joint state
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_names
    joint_state.position = joint_angles
    joint_pub.publish(joint_state)

    broadcaster.sendTransform(
        (robot_x, robot_y, 0),
        tf.transformations.quaternion_from_euler(0, 0, robot_angle),
        rospy.Time.now(),
        'base_link',
        'world')

    robot_x += np.cos(robot_angle)*0.05
    robot_y += np.sin(robot_angle)*0.05
    robot_angle += np.radians(5)
    joint_angles[0] = (joint_angles[0]+np.radians(10))%(2*np.pi) 
    joint_angles[2] = (joint_angles[2]+np.radians(20))%(2*np.pi)
    
    rate.sleep()