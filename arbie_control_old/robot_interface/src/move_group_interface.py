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
import time
from controller_mapper.srv import GetTargetPose, SetTargetPose


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
    group.stop()

def get_target_pose():
    rospy.wait_for_service('get_target_pose')
    try:
        return rospy.ServiceProxy('get_target_pose', GetTargetPose)().target_pose
    except:
        return None

def set_target_pose(target):
    rospy.wait_for_service('set_target_pose')
    try:
        rospy.ServiceProxy('set_target_pose', SetTargetPose)(target)
    except:
        return

def pose_different(pose1, pose2):
    pos_threshold = 0.005
    orient_threshold = 10e-3
    result = abs(pose1.position.x - pose2.position.x) > pos_threshold or \
             abs(pose1.position.y - pose2.position.y) > pos_threshold or \
             abs(pose1.position.z - pose2.position.y) > pos_threshold or \
             abs(pose1.orientation.x - pose2.orientation.x) > orient_threshold or \
             abs(pose1.orientation.y - pose2.orientation.y) > orient_threshold or \
             abs(pose1.orientation.z - pose2.orientation.z) > orient_threshold or \
             abs(pose1.orientation.w - pose2.orientation.w) > orient_threshold
    return result

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = 'arm'
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planning_time(0.1)

    group.set_named_target('ready')
    group.go(wait=True)

    set_target_pose(group.get_current_pose().pose)

    while not rospy.is_shutdown():
        target = get_target_pose()
        if target is not None and pose_different(group.get_current_pose().pose, target):
            success = group.go(target, wait=True)
            group.stop()
            if not success:
                set_target_pose(group.get_current_pose().pose)
                time.sleep(0.01)
        else:
            time.sleep(0.01)


if __name__ == '__main__':
    main()
