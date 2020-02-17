import time
import math

import rospy
import tf
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

target_pose = Pose()
direct_joint_control = True
BASE_SNAP_DIST = 0.1

def get_base_position():
    global tf_listener
    # Use TF transformer to get the initial gripper pose, to use as a base
    # position
    base_trans = None
    try_count = 10
    while base_trans is None:
        try:
            (base_trans, _) = tf_listener.lookupTransform(
                '/world', '/gripper_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            try_count += 1
            time.sleep(0.1)
    base_position = Point()
    base_position.x = base_trans[0]
    base_position.y = base_trans[1]
    base_position.z = base_trans[2]
    return base_position

def init_messages():
    global arm_joints_msg, wrist_joints_msg
    arm_joints_msg = Float32MultiArray()
    arm_joints_msg.layout.dim = [MultiArrayDimension()]
    arm_joints_msg.layout.dim[0].label = "joints"
    arm_joints_msg.layout.dim[0].size = 3
    arm_joints_msg.layout.dim[0].stride = 1
    arm_joints_msg.data = [0, 0, 0]
    wrist_joints_msg = Float32MultiArray()
    wrist_joints_msg.layout.dim = [MultiArrayDimension()]
    wrist_joints_msg.layout.dim[0].label = "joints"
    wrist_joints_msg.layout.dim[0].size = 3
    wrist_joints_msg.layout.dim[0].stride = 1
    wrist_joints_msg.data = [0, 0, 0]

def arm_demand_callback(msg):
    global arm_joints_msg
    if not direct_joint_control:
        arm_joints_msg.data = msg.data

def wrist_demand_callback(msg):
    global wrist_joint_msg
    if not direct_joint_control:
        wrist_joints_msg.data = msg.data

def init_handlers():
    global target_pose_pub, arm_demand_pub, wrist_demand_pub
    global arm_demand_sub, wrist_demand_sub, tf_listener
    target_pose_pub = rospy.Publisher(
        '/target_pose', Pose, queue_size=10)
    arm_demand_pub = rospy.Publisher(
        '/arm_demand_angles', Float32MultiArray, queue_size=10)
    wrist_demand_pub = rospy.Publisher(
        '/wrist_demand_angles', Float32MultiArray, queue_size=10)
    arm_demand_sub = rospy.Subscriber(
        '/arm_demand_angles', Float32MultiArray, arm_demand_callback)
    wrist_demand_sub = rospy.Subscriber(
        '/wrist_demand_angles', Float32MultiArray, wrist_demand_callback)
    tf_listener = tf.TransformListener()

def init():
    """ Must be called after init_node"""
    global base_position
    init_messages()
    init_handlers()
    base_position = get_base_position()

prev_time = time.time()
def elapsed_time():
    global prev_time
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time
    if dt>0.1:
        dt=0.1
    return dt

def get_velocities(message):
    dist_scale = 0.3 # 0.3 m/s
    angle_scale = 2 # 2 rad/s
    return [-message.axes[1] * dist_scale,
            -message.axes[0] * dist_scale,
            (message.axes[5] - message.axes[4]) * dist_scale,
            (message.buttons[15] - message.buttons[14]) * angle_scale,
            (message.buttons[13] - message.buttons[12]) * angle_scale,
            (message.buttons[4] - message.buttons[5]) * angle_scale]

def move_pose(pose, velocities, dt):
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    euler = list(tf.transformations.euler_from_quaternion(quaternion))
    for i in range(3):
        euler[i] += velocities[i+3]*dt
    quaternion = tf.transformations.quaternion_from_euler(*euler)

    pose.position.x = pose.position.x + velocities[0]*dt
    pose.position.y = pose.position.y + velocities[1]*dt
    pose.position.z = pose.position.z + velocities[2]*dt
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def close_to_base():
    return math.hypot(target_pose.position.x, target_pose.position.y) < BASE_SNAP_DIST

def free_movement(velocities, dt):
    global target_pose, direct_joint_control
    target_pose = move_pose(target_pose, velocities, dt)
    absolute_pose = Pose()
    absolute_pose.position.x = target_pose.position.x + base_position.x
    absolute_pose.position.y = target_pose.position.y + base_position.y
    absolute_pose.position.z = target_pose.position.z + base_position.z
    absolute_pose.orientation = target_pose.orientation

    if close_to_base():
        transition_to_constrained()
        direct_joint_control = True
        return

    target_pose_pub.publish(absolute_pose)

def angle_to_target(angle, target, velocity, dt):
    delta = velocity*dt
    if abs(angle-target)<delta:
        angle = target
    elif angle<target:
        angle += delta
    else:
        angle -= delta
    return angle

def constrained_movement(velocities, dt):
    global arm_joints_msg, wrist_joints_msg, target_pose, direct_joint_control

    target_pose.position.z += velocities[2]*dt
    if velocities[0] > 0:
        transition_to_free()
        direct_joint_control = False
        return

    # Doing rough IK for arm angles 1 and 2 from z
    arm_length = 0.42
    angle = math.degrees(math.asin(target_pose.position.z))
    if angle<0:
        angle = 0
    arm_joints_msg.data = [
        arm_joints_msg.data[0],
        angle,
        2*angle
    ]
    arm_demand_pub.publish(arm_joints_msg)

def transition_to_constrained():
    global arm_joints_msg, wrist_joints_msg

    arm_length = 0.42
    angle = math.degrees(math.asin(target_pose.position.z))
    if angle<0:
        angle = 0

    dt = 0.05
    at_target = False
    while not at_target:
        arm_joints_msg.data = [
            angle_to_target(arm_joints_msg.data[0], 0, 80, dt),
            angle_to_target(arm_joints_msg.data[1], angle, 80, dt),
            angle_to_target(arm_joints_msg.data[2], 2*angle, 80, dt),
        ]
        wrist_joints_msg.data = [
            angle_to_target(wrist_joints_msg.data[0], 0, 80, dt),
            angle_to_target(wrist_joints_msg.data[1], 0, 80, dt),
            angle_to_target(wrist_joints_msg.data[2], 0, 80, dt)
        ]
        arm_demand_pub.publish(arm_joints_msg)
        wrist_demand_pub.publish(wrist_joints_msg)
        at_target = arm_joints_msg.data[0] == 0 and \
                    abs(arm_joints_msg.data[1] - angle) < 1.0 and \
                    abs(arm_joints_msg.data[2] - 2*angle) < 1.0 and \
                    wrist_joints_msg.data[0] == 0 and \
                    wrist_joints_msg.data[1] == 0 and \
                    wrist_joints_msg.data[2] == 0
        time.sleep(dt)

def transition_to_free():
    global arm_joints_msg, wrist_joints_msg
    dt = 0.05
    at_target = False
    while not at_target:
        arm_joints_msg.data = [
            arm_joints_msg.data[0],
            angle_to_target(arm_joints_msg.data[1], 60, 80, dt),
            angle_to_target(arm_joints_msg.data[2], 60, 80, dt),
        ]
        arm_demand_pub.publish(arm_joints_msg)
        at_target = abs(arm_joints_msg.data[1] - 60) < 1.0 and \
                    abs(arm_joints_msg.data[2] - 60) < 1.0
        time.sleep(dt)
    # Reset target pose
    success = False
    try_count = 0
    while try_count < 10 and not success:
        try:
            (trans , rot) = tf_listener.lookupTransform(
                '/world', '/gripper_1', rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            time.sleep(0.1)
            try_count += 1
    if not success:
        print("TF lookup failed")
    target_pose.position.x = trans[0] - base_position.x
    target_pose.position.y = trans[1] - base_position.y
    target_pose.position.z = trans[2] - base_position.z
    target_pose.orientation.x = rot[0]
    target_pose.orientation.y = rot[1]
    target_pose.orientation.z = rot[2]
    target_pose.orientation.w = rot[3]


def update(message):
    global target_pose, direct_joint_control, arm_joints_msg
    dt = elapsed_time()
    velocities = get_velocities(message)
    if direct_joint_control:
        constrained_movement(velocities, dt)
    else:
        free_movement(velocities, dt)

