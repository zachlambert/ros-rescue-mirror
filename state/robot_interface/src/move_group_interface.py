#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import tf


def set_pose(group, x, y, z, roll, pitch, yaw):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    group.go(pose_goal, wait=True)
    group.stop()

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

    group.go(pose_goal, wait=True)
    #group.stop()

def move_to_random_pose(group):
    pose_goal = group.get_random_pose('gripper_1')
    group.go(pose_goal, wait=True)
    group.stop()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = 'arm'
    group = moveit_commander.MoveGroupCommander(group_name)

    planning_frame = group.get_planning_frame()
    print 'Reference Frame: %s' % planning_frame
    eef_link = group.get_end_effector_link()
    print 'End Effector Link: %s' % eef_link
    group_names = robot.get_group_names()
    print 'Robot Groups: ', group_names

    set_pose(group, 0.4, 0, 0.4, pi/2, 0, 0)

    freq = 4
    rate = rospy.Rate(freq)
    count = 0
    vx = 0.2
    time = 2
    while not rospy.is_shutdown() and count<time*freq:
        move_pose(group, vx/freq, 0, 0, 0, 0, 0)
        count += 1
        rate.sleep()

    """ 
    user_input = raw_input('>')
    while user_input != 'exit':
        inputs = user_input.split(' ')
        outputs = [0, 0, 0, 0, 0, 0]
        for i in range(0, len(inputs)):
            try:
                outputs[i] = float(inputs[i])
            except:
                pass
        move_pose(group, *outputs)
    """

if __name__ == '__main__':
    main()
