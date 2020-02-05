#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import tf
import math

arm_values = {"r": 0.1, "theta": 0, "z": 0.1}

arm_msg = Float32MultiArray()


def receive_joy_data(message):
    global odrive_pub
    drive_left.publish(-(3 * message.axes[1] * 8192 * 16 + message.axes[0] * 16 * 8192))
    drive_right.publish(3 * message.axes[1] * 8192 * 16 - message.axes[0] * 16 * 8192)
    flipper_front.publish((message.buttons[12] - message.buttons[13]) * 8192 * 85 / 8)
    flipper_rear.publish((message.buttons[15] - message.buttons[14]) * 8192 * 85 / 8)
    if message.buttons[4]:
        arm_values["theta"] += 0.002
    elif message.buttons[5]:
        arm_values["theta"] -= 0.002

    if message.buttons[0]:
        arm_values["r"] += 0.02
    elif message.buttons[3]:
        arm_values["r"] -= 0.02

    if message.buttons[1]:
        arm_values["z"] += 0.02
    elif message.buttons[2]:
        arm_values["z"] -= 0.02

    # print(repr(arm_values))

    set_pose_cylindrical(group, arm_values["r"], arm_values["theta"], arm_values["z"], pi / 2, 0, 0)


def receive_arm_data(message):
    global arm_values
    arm_values = message.data


def set_pose_cartesian(group, x, y, z, roll, pitch, yaw):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    # print(x, ":", y, ":", z)

    group.go(pose_goal, wait=False)
    # group.stop()


def set_pose_cylindrical(group, r, theta, z, roll, pitch, yaw):
    x = r * math.cos(theta)
    y = r * math.sin(theta)

    set_pose_cartesian(group, x, y, z, roll, pitch, yaw)


moveit_commander.roscpp_initialize(sys.argv)

group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

# base button 5 and 4
rospy.init_node("controller_mapper")

rospy.Subscriber("/gamepad/values", Joy, receive_joy_data)
# rospy.Subscriber("/arm_current_values", Float32MultiArray, receive_arm_data)

drive_left = rospy.Publisher('/odrive_drive/axis0/vel_setpoint', Int32, queue_size=10)
drive_right = rospy.Publisher('/odrive_drive/axis1/vel_setpoint', Int32, queue_size=10)
flipper_front = rospy.Publisher('/odrive_flipper/axis0/vel_setpoint', Int32, queue_size=10)
flipper_rear = rospy.Publisher('/odrive_flipper/axis1/vel_setpoint', Int32, queue_size=10)
arm = rospy.Publisher('/arm_demand_angles', Float32MultiArray, queue_size=10)

rospy.spin()
