#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState

rospy.init_node('robot_pose_publisher')

# Initialisation

def get_initial_robot_pose():
    pose = Pose()
    pose.position.x=0
    pose.position.y=0
    pose.position.z=0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


robot_pose = get_initial_robot_pose()
tf_broadcaster = tf.TransformBroadcaster()

base_transform = TransformStamped()
base_transform.header.frame_id = 'world'
base_transform.child_frame_id = 'base_link'

# Pose callback

def callback(pose):
    global robot_pose
    robot_pose = pose

rospy.Subscriber('robot_pose', Pose, callback)

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    base_transform.header.stamp = rospy.Time.now()
    base_transform.transform.translation = robot_pose.position
    base_transform.transform.rotation = robot_pose.orientation
    tf_broadcaster.sendTransformMessage(base_transform)
    rate.sleep()
