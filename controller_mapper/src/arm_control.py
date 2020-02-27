import time
import math

import rospy
import tf
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float32

""" Global variables declared early """

target_pose = Pose()
direct_joint_control = True
BASE_SNAP_DIST = 0.1


""" Functions for directly controlling the joints """


def angles_step_to_target(angles, targets, velocities, dt):
    finished = True
    for i in range(len(angles)):
        delta = velocities[i] * dt
        if abs(angles[i]-targets[i]) < delta:
            angles[i] = targets[i]
            continue
        elif angles[i]<targets[i]:
            angles[i] += delta
        else:
            angles[i] -= delta
        finished = False
    return angles, finished


def move_to_joint_target(targets, velocities):
    global arm_joints_msg, wrist_joints_msg
    angles = list(arm_joints_msg.data) + list(wrist_joints_msg.data)
    dt = 0.05
    finished = False
    while not finished:
        angles, finished = angles_step_to_target(angles, targets, velocities, dt)
        arm_joints_msg.data = angles[:3]
        wrist_joints_msg.data = angles[3:]
        arm_demand_pub.publish(arm_joints_msg)
        wrist_demand_pub.publish(wrist_joints_msg)
        time.sleep(dt)


def calibrate_target_pose():
    global target_pose
    # Reset target pose
    pos, rot = get_gripper_position()
    target_pose.position.x = pos[0] - base_position.x
    target_pose.position.y = pos[1] - base_position.y
    target_pose.position.z = pos[2] - base_position.z
    target_pose.orientation.x = rot[0]
    target_pose.orientation.y = rot[1]
    target_pose.orientation.z = rot[2]
    target_pose.orientation.w = rot[3]


def transition_to_constrained():
    arm_length = 0.42
    angle = math.degrees(math.asin(target_pose.position.z))
    if angle<0:
        angle = 0

    move_to_joint_target([0, angle, 2*angle, 0, 0, 0],
                         [80, 80, 80, 80, 80, 80])


def transition_to_free():
    global arm_joints_msg, wrist_joints_msg
    move_to_joint_target([arm_joints_msg.data[0], 60, 60] + list(wrist_joints_msg.data),
                         [80, 80, 80, 80, 80, 80])
    calibrate_target_pose()


def reset_gripper_orientation():
    global arm_joints_msg, wrist_joints_msg
    move_to_joint_target(list(arm_joints_msg.data) + [0, 0, 0],
                         [80, 80, 80, 80, 80, 80])
    calibrate_target_pose()


""" TF information """


def get_gripper_position():
    global tf_listener
    while True:
        try:
            (trans, rot) = tf_listener.lookupTransform('/world', '/gripper_1', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            time.sleep(0.1)
    return trans, rot


""" Callback functions """


def arm_demand_callback(msg):
    global arm_joints_msg
    if not direct_joint_control:
        arm_joints_msg.data = msg.data


def wrist_demand_callback(msg):
    global wrist_joint_msg
    if not direct_joint_control:
        wrist_joints_msg.data = msg.data


""" Initialisation """


def init_messages():
    global arm_joints_msg, wrist_joints_msg, zero_arm_msg

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

    zero_arm_msg = Bool()
    zero_arm_msg.data = True


def init_handlers():
    global target_pose_pub, arm_demand_pub, wrist_demand_pub
    global arm_demand_sub, wrist_demand_sub
    global tf_listener, zero_arm_pub
    target_pose_pub = rospy.Publisher(
        '/target_pose', Pose, queue_size=10)
    arm_demand_pub = rospy.Publisher(
        '/arm_demand_angles', Float32MultiArray, queue_size=10)
    wrist_demand_pub = rospy.Publisher(
        '/wrist_demand_angles', Float32MultiArray, queue_size=10)
    gripper_velocity_pub = rospy.Publisher(
        '/gripper/velocity', Float32, queue_size=10)
    arm_demand_sub = rospy.Subscriber(
        '/arm_demand_angles', Float32MultiArray, arm_demand_callback)
    wrist_demand_sub = rospy.Subscriber(
        '/wrist_demand_angles', Float32MultiArray, wrist_demand_callback)
    tf_listener = tf.TransformListener()
    zero_arm_pub = rospy.Publisher(
        '/zero_arm_request', Bool, queue_size=1)


""" For measuring elapsed time since the last function call """

prev_time = time.time()
def elapsed_time():
    global prev_time
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time
    if dt>0.1:
        dt=0.1
    return dt


""" Functions for helping with movement """


def get_velocities(message):
    dist_scale = 0.3 # 0.3 m/s
    angle_scale = 2 # 2 rad/s
    return [-message.axes[1] * dist_scale,
            -message.axes[0] * dist_scale,
            (message.axes[5] - message.axes[4]) * dist_scale,
            message.axes[2] * angle_scale,
            -message.axes[3] * angle_scale,
            #(message.buttons[15] - message.buttons[14]) * angle_scale,
            #(message.buttons[13] - message.buttons[12]) * angle_scale,
            (message.buttons[4] - message.buttons[5]) * angle_scale]


control_mode = 2


# 0: Cartesian, 1: Polar, 2: Relative to gripper orientation
def move_pose(pose, velocities, dt):
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    euler = list(tf.transformations.euler_from_quaternion(quaternion))
    for i in range(3):
        euler[i] += velocities[i+3]*dt

    if control_mode == 0:
        pose.position.x = pose.position.x + velocities[0]*dt
        pose.position.y = pose.position.y + velocities[1]*dt
    else:
        if control_mode == 1:
            euler[2] = math.atan2(pose.position.y, pose.position.x)
        yaw = -euler[2]
        pose.position.x = pose.position.x + velocities[0]*dt*math.cos(yaw) \
                                          + velocities[1]*dt*math.sin(yaw)
        pose.position.y = pose.position.y - velocities[0]*dt*math.sin(yaw) \
                                          + velocities[1]*dt*math.cos(yaw)

    pose.position.z = pose.position.z + velocities[2]*dt

    quaternion = tf.transformations.quaternion_from_euler(*euler)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def is_close_to_base():
    return math.hypot(target_pose.position.x, target_pose.position.y) < BASE_SNAP_DIST


def zero_arm():
    global direct_joint_control, target_pose, arm_joints_msg
    zero_arm_pub.publish(zero_arm_msg)
    direct_joint_control = True
    target_pose.position.x = 0
    target_pose.position.y = 0
    target_pose.position.z = 0
    arm_joints_msg.data = [0, 0, 0]


""" Movement control functions: constrained or free """


def free_movement(velocities, dt):
    global target_pose, direct_joint_control
    target_pose = move_pose(target_pose, velocities, dt)
    absolute_pose = Pose()
    absolute_pose.position.x = target_pose.position.x + base_position.x
    absolute_pose.position.y = target_pose.position.y + base_position.y
    absolute_pose.position.z = target_pose.position.z + base_position.z
    absolute_pose.orientation = target_pose.orientation

    if is_close_to_base():
        transition_to_constrained()
        direct_joint_control = True
        return

    target_pose_pub.publish(absolute_pose)


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


""" Handle buttons """
button_status = [False, False, False]


def handle_buttons(message):
    global button_status, control_mode, zero_arm_pub, zero_arm_msg
    # 0 -> A
    if message.buttons[0] == 1 and not button_status[0]:
        button_status[0] = True
        reset_gripper_orientation()
    elif message.buttons[0] == 0 and button_status[0]:
        button_status[0] = False
    # 2 -> X
    if message.buttons[2] == 1 and not button_status[1]:
        button_status[1] = True
        control_mode = (control_mode+1)%3
    elif message.buttons[2] == 0 and button_status[1]:
        button_status[1] = False
    # 3 -> Y
    if message.buttons[3] == 1 and not button_status[2]:
        button_status[2] = True
        zero_arm()
    elif message.buttons[3] == 0 and button_status[2]:
        button_status[2] = False


""" "Public" Functions """


def init():
    global base_position
    init_messages()
    init_handlers()

    base_trans, _ = get_gripper_position()
    base_position = Point()
    base_position.x = base_trans[0]
    base_position.y = base_trans[1]
    base_position.z = base_trans[2]


def update(message):
    global target_pose, direct_joint_control, arm_joints_msg
    dt = elapsed_time()
    velocities = get_velocities(message)
    handle_buttons(message)
    if direct_joint_control:
        constrained_movement(velocities, dt)
    else:
        free_movement(velocities, dt)

