#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState

# Callback functions

robot_x = 0
robot_y = 0
robot_angle = 0

joints = {
    'flipper_front_right': 0,
    'flipper_front_left': 0,
    'flipper_rear_right': 0,
    'flipper_rear_left': 0,
    'arm_pan': 0,
    'arm_lower_joint': 0,
    'arm_upper_joint': 0
}

def _position_callback(data):
    global robot_x, robot_y, robot_angle
    robot_x = data.x
    robot_y = data.y
    robot_angle = data.angle

def _arm_angles_callback(data):
    joints['arm_pan'] = data.x
    joints['arm_lower_joint'] = data.y
    joints['arm_upper_joint'] = data.z

def _flippers_callback(data):
    front_angle = data.data[0]
    rear_angle = data.data[1]
    joints['flipper_front_right'] = front_angle
    joints['flipper_front_left'] = front_angle
    joints['flipper_rear_right'] = rear_angle
    joints['flipper_rear_left'] = rear_angle


# Setup node

rospy.init_node('state_publisher')

rospy.Subscriber('arm/angles', Vector3, _arm_angles_callback)
rospy.Subscriber('flipper/position', Float32MultiArray, _flippers_callback)

joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

# Publish position and joints

broadcaster = tf.TransformBroadcaster()

joint_state = JointState()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # Update joint state
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joints.keys()
    joint_state.position = joints.values()
    joint_pub.publish(joint_state)

    broadcaster.sendTransform(
        (robot_x, robot_y, 0),
        tf.transformations.quaternion_from_euler(0, 0, robot_angle),
        rospy.Time.now(),
        'base_link',
        'world')

    rate.sleep()
