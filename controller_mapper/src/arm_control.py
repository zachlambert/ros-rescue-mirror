import time
import math

import rospy
import tf
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float, String

from inverse_kinematics.srv import PoseToAngles, AnglesToPose

""" Global variables declared early """

pose = Pose()
arm_angles = [0 for i in range(3)]
wrist_angles = [0 for i in range(3)]
direct_joint_control = True

""" Functions for writing and publishing joint angles """

def set_pose(new_pose):
    global pose, arm_angles, wrist_angles, arm_joints_msg, wrist_joints_msg
    response = pose_to_angles(convert_pose_to_absolute(new_pose))
    if response.valid:
        pose = new_pose
        arm_angles = list(response.arm_angles.data)
        wrist_angles = list(response.wrist_angles.data)
        arm_joints_msg.data = arm_angles
        wrist_joints_msg.data = wrist_angles
        publish_angles()
        return True
    else:
        return False

def set_angles(new_arm_angles, new_wrist_angles):
    global arm_angles, wrist_angles, arm_joints_msg, wrist_joints_msg, pose
    arm_joints_msg.data = new_arm_angles
    wrist_joints_msg.data = new_wrist_angles
    response = angles_to_pose(arm_joints_msg, wrist_joints_msg)
    if response.valid:
        arm_angles = new_arm_angles
        wrist_angles = new_wrist_angles
        pose = convert_pose_to_relative(response.pose)
        publish_angles()
        return True
    else:
        return False

def publish_angles():
    global arm_demand_pub, wrist_demand_pub, arm_joints_msg, wrist_joints_msg
    arm_demand_pub.publish(arm_joints_msg)
    wrist_demand_pub.publish(wrist_joints_msg)

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
    global arm_angles, wrist_angles
    angles = list(arm_angles + wrist_angles)
    dt = 0.05
    finished = False
    while not finished:
        angles, finished = angles_step_to_target(angles, targets, velocities, dt)
        valid = set_angles(angles[:3], angles[3:])
        if not valid:
            return
        time.sleep(dt)

def transition_to_constrained():
    global pose, direct_joint_control
    arm_length = 0.42
    angle = math.degrees(math.asin(pose.position.z))
    if angle<0:
        angle = 0
    move_to_joint_target([0, angle, 2*angle, 0, 0, 0],
                         [80, 80, 80, 80, 80, 80])
    direct_joint_control = False

def transition_to_free():
    global arm_angles, wrist_angles, direct_joint_control
    move_to_joint_target([arm_angles[0], 80, 50] + wrist_angles,
                         [80, 80, 80, 80, 80, 80])
    direct_joint_control = True

def reset_gripper_orientation():
    global arm_angles, wrist_angles
    move_to_joint_target(arm_angles + [0, 0, 0],
                         [80, 80, 80, 80, 80, 80])


""" TF information """

def convert_pose_to_absolute(relative):
    global base_position
    absolute = Pose()
    absolute.position.x = base_position.x + relative.position.x
    absolute.position.y = base_position.y + relative.position.y
    absolute.position.z = base_position.z + relative.position.z
    absolute.orientation = relative.orientation
    return absolute

def convert_pose_to_relative(absolute):
    global base_position
    relative = Pose()
    relative.position.x = absolute.position.x - base_position.x
    relative.position.y = absolute.position.y - base_position.y
    relative.position.z = absolute.position.z - base_position.z
    relative.orientation = absolute.orientation
    return relative

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
    global target_pose_pub, arm_demand_pub, wrist_demand_pub, arm_control_mode_pubtf_listener, zero_arm_pub
    arm_demand_pub = rospy.Publisher(
        '/arm_demand_angles', Float32MultiArray, queue_size=10)
    wrist_demand_pub = rospy.Publisher(
        '/wrist_demand_angles', Float32MultiArray, queue_size=10)
    gripper_velocity_pub = rospy.Publisher(
        '/gripper/velocity', Float32, queue_size=10)
    tf_listener = tf.TransformListener()
    zero_arm_pub = rospy.Publisher(
        '/zero_arm_request', Bool, queue_size=1)
    arm_control_mode_pub = rospy.Publisher(
        '/arm_control_mode', String, queue_size=1)

def init_service_proxies():
    global pose_to_angles, angles_to_pose
    rospy.wait_for_service("pose_to_angles")
    rospy.wait_for_service("angles_to_pose")
    try:
        pose_to_angles = rospy.ServiceProxy("pose_to_angles", PoseToAngles)
        angles_to_pose = rospy.ServiceProxy("angles_to_pose", AnglesToPose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

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
            (message.buttons[4] - message.buttons[5]) * angle_scale]

control_mode = 1
#0: Cartesian, 1: Polar, 2: Relative to gripper orientation
def move_pose(pose, velocities, dt):
    new_pose = Pose()
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    euler = list(tf.transformations.euler_from_quaternion(quaternion))
    for i in range(3):
        euler[i] += velocities[i+3]*dt

    if control_mode == 0:
        new_pose.position.x = pose.position.x + velocities[0]*dt
        new_pose.position.y = pose.position.y + velocities[1]*dt
    else:
        if control_mode == 1:
            euler[2] = math.atan2(pose.position.y, pose.position.x)
        yaw = -euler[2]
        new_pose.position.x = pose.position.x + velocities[0]*dt*math.cos(yaw) \
                                              + velocities[1]*dt*math.sin(yaw)
        new_pose.position.y = pose.position.y - velocities[0]*dt*math.sin(yaw) \
                                              + velocities[1]*dt*math.cos(yaw)

    new_pose.position.z = pose.position.z + velocities[2]*dt

    quaternion = tf.transformations.quaternion_from_euler(*euler)
    new_pose.orientation.x = quaternion[0]
    new_pose.orientation.y = quaternion[1]
    new_pose.orientation.z = quaternion[2]
    new_pose.orientation.w = quaternion[3]
    return new_pose

def is_close_to_base(pose):
    return math.hypot(pose.position.x, pose.position.y) < 0.05

def zero_arm():
    global direct_joint_control, pose, arm_joints_msg
    zero_arm_pub.publish(zero_arm_msg)
    direct_joint_control = True
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    arm_joints_msg.data = [0, 0, 0]

""" Movement control functions: constrained or free """

def free_movement(velocities, dt):
    global pose, direct_joint_control, base_position
    new_pose = move_pose(pose, velocities, dt)

    if is_close_to_base(new_pose):
        transition_to_constrained()
        direct_joint_control = True
        publish_arm_control_mode()
    else:
        set_pose(new_pose)

def constrained_movement(velocities, dt):
    global arm_angles, wrist_angles, pose, direct_joint_control

    if velocities[0] > 0:
        transition_to_free()
        direct_joint_control = False
        publish_arm_control_mode()
        return

    arm_length = 0.42
    current_angle = arm_angles[1]
    z_estimate = 2*arm_length*math.sin(math.radians(current_angle))
    z_estimate += velocities[2]*dt
    new_angle = math.degrees(math.asin(0.5*z_estimate/arm_length))
    if new_angle<0:
        new_angle = 0
    new_arm_angles = [arm_angles[0], new_angle, new_angle*2]
    print(new_arm_angles)
    set_angles(new_arm_angles, wrist_angles)

def publish_arm_control_mode():
    string = ""
    if direct_joint_control:
        string += "joint control"
    else:
        string += "pose control : "
        string += ["cartesian", "polar", "relative to gripper"][control_mode]
    arm_control_mode_pub.publish(string)

""" Handle buttons """

button_status = [False, False, False, False, False, False]
def handle_buttons(message):
    global button_status, control_mode, zero_arm_pub, zero_arm_msg, gripper_velocity_pub
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
        publish_arm_control_mode()
    elif message.buttons[2] == 0 and button_status[1]:
        button_status[1] = False
    # 3 -> Y
    if message.buttons[3] == 1 and not button_status[2]:
        button_status[2] = True
        zero_arm()
    elif message.buttons[3] == 0 and button_status[2]:
        button_status[2] = False
    # D-Pad buttons

    if message.buttons[15] == 1 and not button_status[3]:
        button_status[3] = True
        change_roll(90)
    elif message.buttons[15] == 0 and button_status[3]:
        button_status[3] = False

    if message.buttons[14] == 1 and not button_status[4]:
        button_status[4] = True
        change_roll(-90)
    elif message.buttons[14] == 0 and button_status[4]:
        button_status[4] = False


def change_roll(change):
    global arm_angles, wrist_angles
    new_roll = wrist_angles[2] + change
    new_roll = (new_roll + 180)%360 - 180
    set_angles(arm_angles, [wrist_angles[0], wrist_angles[1], new_roll])

""" "Public" Functions """

def init():
    global base_position, pose, arm_angles, wrist_angles, arm_joints_msg, wrist_joints_msg
    init_messages()
    init_handlers()
    init_service_proxies()

    arm_joints_msg.data = arm_angles
    wrist_joints_msg.data = wrist_angles
    response = angles_to_pose(arm_joints_msg, wrist_joints_msg)
    pose = response.pose
    base_position = pose.position

def update(message):
    global pose, direct_joint_control, arm_joints_msg
    dt = elapsed_time()
    velocities = get_velocities(message)
    handle_buttons(message)
    if direct_joint_control:
        constrained_movement(velocities, dt)
    else:
        free_movement(velocities, dt)

