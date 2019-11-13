#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32


def receive_data(message):
    global odrive_pub
    print(message.axes[1])
    drive_left.publish(message.axes[1] * 8192)
    drive_right.publish(message.axes[1] * 8192)


rospy.init_node("controller_mapper")

rospy.Subscriber("/gamepad/values", Joy, receive_data)

drive_left = rospy.Publisher('/odrive_drive/axis0/vel_setpoint', Int32, queue_size=10)
drive_right = rospy.Publisher('/odrive_drive/axis1/vel_setpoint', Int32, queue_size=10)

rospy.spin()