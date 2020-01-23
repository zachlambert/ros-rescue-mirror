import rospy
import tf
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

import state_publisher

def _pose_cb(pose):
    #If need to convert into pose format:
    # pose = Pose()
    # pose.position.x = x
    # ...
    # pose.orientation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    state_publisher.set_pose(pose)

def _flippers_position_cb(positions):
    # Check what the data format is of positions
    state_publisher.set_flipper_positions(positions.data[0], positions.data[1])

def _arm_angles_cb(angles):
    state_publisher.set_arm_angles(angles.data[0], angles.data[1], angles.data[2])

def init():

    rospy.Subscriber("pose", Pose, _pose_cb)
    rospy.Subscriber("flippers/position", Float32MultiArray, _flippers_position_cb)
    rospy.Subscriber("arm/angles", Float32MultiArray, _arm_angles_cb)

