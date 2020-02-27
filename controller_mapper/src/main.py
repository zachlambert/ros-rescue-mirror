#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import sys
from math import pi
from std_msgs.msg import String

import arm_control

arm_control_mode = True
last_message = None


def receive_joy_data(message):
    global arm_control_mode, last_message, target_pose

    if message.buttons[16] == 1 and last_message == 0:
        arm_control_mode = not arm_control_mode
    last_message = message.buttons[16]

    if arm_control_mode:
        arm_control.update(message)
    else:
        drive_left.publish(-(message.axes[1] * 8192 * 32 + message.axes[0] * 32 * 8192))
        drive_right.publish(message.axes[1] * 8192 * 32 - message.axes[0] * 32 * 8192)
        flipper_front.publish((message.buttons[12] - message.buttons[13]) * 8192 * 85 / 8)
        flipper_rear.publish((message.buttons[15] - message.buttons[14]) * 8192 * 85 / 8)

# base button 5 and 4
rospy.init_node("controller_mapper")

rospy.Subscriber("/gamepad/values", Joy, receive_joy_data)

drive_left = rospy.Publisher('/odrive_drive/axis0/vel_setpoint', Int32, queue_size=10)
drive_right = rospy.Publisher('/odrive_drive/axis1/vel_setpoint', Int32, queue_size=10)
flipper_front = rospy.Publisher('/odrive_flipper/axis0/vel_setpoint', Int32, queue_size=10)
flipper_rear = rospy.Publisher('/odrive_flipper/axis1/vel_setpoint', Int32, queue_size=10)

arm_control.init()

rospy.spin()
