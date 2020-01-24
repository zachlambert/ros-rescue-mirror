import rospy
import tf

from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState

# Callback functions


joints = {
    'Rev3': 0,
    'Rev4': 0,
    'Rev7': 0,
    'Rev8': 0,
    'Rev9': 0
}

def set_pose(pose):
    global robot_pose
    robot_pose = pose

def set_flipper_positions(front_angle, rear_angle):
    joints['Rev3'] = front_angle
    joints['Rev4'] = rear_angle

def set_arm_angles(angle1, angle2, angle3):
    joints['Rev7'] = angle1
    joints['Rev8'] = angle2
    joints['Rev9'] = angle3

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

def init():
    global joint_state, robot_pose, tf_broadcaster, base_transform, joint_pub
    joint_state = JointState()

    robot_pose = get_initial_robot_pose()
    tf_broadcaster = tf.TransformBroadcaster()

    base_transform = TransformStamped()
    base_transform.header.frame_id = 'world'
    base_transform.child_frame_id = 'base_link'

    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

def publish():
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joints.keys()
    joint_state.position = joints.values()
    joint_pub.publish(joint_state)

    base_transform.header.stamp = rospy.Time.now()
    base_transform.transform.translation = robot_pose.position
    base_transform.transform.rotation = robot_pose.orientation

    tf_broadcaster.sendTransformMessage(base_transform)
