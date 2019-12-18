#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

arm_values = [0, 0, 0]

arm_msg = Float32MultiArray()


def receive_joy_data(message):
    global odrive_pub
    drive_left.publish(-(message.axes[1] * 8192 * 16 + message.axes[0] * 16 * 8192))
    drive_right.publish(message.axes[1] * 8192 * 16 - message.axes[0] * 16 * 8192)
    flipper_front.publish((message.buttons[12] - message.buttons[13]) * 8192 * 85 / 4)
    flipper_rear.publish((message.buttons[15] - message.buttons[14]) * 8192 * 85 / 4)
    if message.buttons[4]:
        arm_values[0] += 0.2
    elif message.buttons[5]:
        arm_values[0] -= 0.2

    if message.buttons[0]:
        arm_values[1] += 0.2
    elif message.buttons[3]:
        arm_values[1] -= 0.2

    if message.buttons[1]:
        arm_values[2] += 0.2
    elif message.buttons[2]:
        arm_values[2] -= 0.2

    arm_msg.data = arm_values
    arm.publish(arm_msg)


def receive_arm_data(message):
    global arm_values
    arm_values = message.data

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
