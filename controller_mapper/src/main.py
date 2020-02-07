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

arm_control_mode = True
last_message = None


def receive_joy_data(message):
    global arm_control_mode, last_message

    if message.buttons[16] == 1 and last_message == 0:
        arm_control_mode = not arm_control_mode
    last_message = message.buttons[16]

    if arm_control_mode:
        if message.buttons[8]:
            pass  # home
        elif message.buttons[9]:
            pass  # ready
        else:
            dist = 0.01
            ang = 0.01

            angles = ["yaw", "pitch", "roll"]

            moves = {"x": -message.axes[1],
                     "y": message.axes[0],
                     "z": message.axes[5] - message.axes[4],
                     "yaw": message.buttons[4] - message.buttons[5],
                     "roll": message.buttons[14] - message.buttons[15],
                     "pitch": message.buttons[12] - message.buttons[13]
                     }

            # for key, value in moves.items():
            #     if key in angles:
            #         moves[key] *= ang
            #     else:
            #         moves[key] *= dist
            print(message)
            print(moves)
            move_pose(group, moves["x"], moves["y"], moves["z"], moves["yaw"], moves["roll"], moves["pitch"])
    else:
        drive_left.publish(-(3 * message.axes[1] * 8192 * 16 + message.axes[0] * 16 * 8192))
        drive_right.publish(3 * message.axes[1] * 8192 * 16 - message.axes[0] * 16 * 8192)
        flipper_front.publish((message.buttons[12] - message.buttons[13]) * 8192 * 85 / 8)
        flipper_rear.publish((message.buttons[15] - message.buttons[14]) * 8192 * 85 / 8)


def move_pose(group, x, y, z, roll, pitch, yaw):
    pose = group.get_current_pose().pose
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    euler = (euler[0] + roll,
             euler[1] + pitch,
             euler[2] + yaw)
    quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose.position.x + x
    pose_goal.position.y = pose.position.y + y
    pose_goal.position.z = pose.position.z + z
    #pose_goal.orientation.w = 1
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    group.go(pose_goal, wait=False)


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

rospy.spin()
