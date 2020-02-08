#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import sys
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf
import math
import time

arm_msg = Float32MultiArray()

arm_control_mode = True
last_message = None

target_pose = Pose()
prev_time = time.time()

def move_pose(pose, v_x, v_y, v_z, v_roll, v_pitch, v_yaw):
    global prev_time

    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time
    if dt>0.1:
        dt=0.1

    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    euler = (euler[0] + v_roll*dt,
             euler[1] + v_pitch*dt,
             euler[2] + v_yaw*dt)
    quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

    pose.position.x = pose.position.x + v_x*dt
    pose.position.y = pose.position.y + v_y*dt
    pose.position.z = pose.position.z + v_z*dt
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def receive_joy_data(message):
    global arm_control_mode, last_message, target_pose

    if message.buttons[16] == 1 and last_message == 0:
        arm_control_mode = not arm_control_mode
    last_message = message.buttons[16]

    if arm_control_mode:
        if message.buttons[8]:
            pass  # home
        elif message.buttons[9]:
            pass  # ready
        else:
            dist = 0.3
            ang = 2

            angles = ["yaw", "pitch", "roll"]

            moves = {"x": -message.axes[1],
                     "y": -message.axes[0],
                     "z": message.axes[5] - message.axes[4],
                     "yaw": message.buttons[4] - message.buttons[5],
                     "roll": message.buttons[15] - message.buttons[14],
                     "pitch": message.buttons[13] - message.buttons[12]
                     }

            for key, value in moves.items():
                if key in angles:
                    moves[key] *= ang
                else:
                    moves[key] *= dist

            target_pose = move_pose(
                target_pose,
                moves['x'],
                moves['y'],
                moves['z'],
                moves['roll'],
                moves['pitch'],
                moves['yaw'])
            target_pose_pub.publish(target_pose)
            print(target_pose)
    else:
        drive_left.publish(-(3 * message.axes[1] * 8192 * 16 + message.axes[0] * 16 * 8192))
        drive_right.publish(3 * message.axes[1] * 8192 * 16 - message.axes[0] * 16 * 8192)
        flipper_front.publish((message.buttons[12] - message.buttons[13]) * 8192 * 85 / 8)
        flipper_rear.publish((message.buttons[15] - message.buttons[14]) * 8192 * 85 / 8)

# base button 5 and 4
rospy.init_node("controller_mapper")

rospy.Subscriber("/gamepad/values", Joy, receive_joy_data)
# rospy.Subscriber("/arm_current_values", Float32MultiArray, receive_arm_data)

drive_left = rospy.Publisher('/odrive_drive/axis0/vel_setpoint', Int32, queue_size=10)
drive_right = rospy.Publisher('/odrive_drive/axis1/vel_setpoint', Int32, queue_size=10)
flipper_front = rospy.Publisher('/odrive_flipper/axis0/vel_setpoint', Int32, queue_size=10)
flipper_rear = rospy.Publisher('/odrive_flipper/axis1/vel_setpoint', Int32, queue_size=10)

target_pose_pub = rospy.Publisher('/target_pose', Pose, queue_size=10)

rospy.spin()
