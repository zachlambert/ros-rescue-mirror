import rospy
import tf

from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState

# Callback functions

robot_pose = Pose()

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

def initialise_robot_pose():
    global robot_pose
    robot_pose.position.x=0
    robot_pose.position.y=0
    robot_pose.position.z=0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    robot_pose.orientation.x = quaternion[0]
    robot_pose.orientation.y = quaternion[1]
    robot_pose.orientation.z = quaternion[2]
    robot_pose.orientation.w = quaternion[3]

def run():
    initialise_robot_pose()

    broadcaster = tf.TransformBroadcaster()
    joint_state = JointState()

    transform = TransformStamped()
    transform.header.frame_id = 'world'
    transform.child_frame_id = 'base_link'

    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Update joint state
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = joints.keys()
        joint_state.position = joints.values()
        joint_pub.publish(joint_state)

        transform.header.stamp = rospy.Time.now()
        transform.transform.translation = robot_pose.position
        transform.transform.rotation = robot_pose.orientation

        broadcaster.sendTransformMessage(transform)

        rate.sleep()
