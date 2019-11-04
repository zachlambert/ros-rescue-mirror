import rospy  # in PyCharm shows error, solution: https://gavazzi.us/2017/07/31/pycharm-ros/
from std_msgs import msg

import drive


# SUBSCRIBER CALLBACKS


def _commands_cb(command):
    try:
        if chr(command.data) == 'r':
            drive.full_reset_and_calibrate_all()
        if chr(command.data) == 's':
            drive.stop_all()
    except Exception as e:
        debug_publish("ERROR in commands: " + str(e))
    rospy.loginfo("Recieved on commands: " + str(command.data))


def _drive_distance_cb(distances):
    try:
        drive.drive_distance(distances.data[0], distances.data[1])
    except Exception as e:
        debug_publish("ERROR in drive/distance: " + str(e))
    rospy.loginfo("Recieved on drive/distance: " + ' '.join(map(str, distances.data)))


def _drive_velocity_cb(velocities):
    try:
        drive.drive_velocity(velocities.data[0], velocities.data[1])
    except Exception as e:
        debug_publish("ERROR in drive/velocity: " + str(e))
    rospy.loginfo("Recieved on drive/velocity: " + ' '.join(map(str, velocities.data)))


def _flippers_position_cb(positions):
    try:
        drive.flipper_position(positions.data[0], positions.data[1], positions.data[2], positions.data[3])
    except Exception as e:
        debug_publish("ERROR in flippers/position: " + str(e))
    rospy.loginfo("Recieved on flippers/position: " + ' '.join(map(str, positions.data)))


def _flipper_velocity_cb(velocities):
    try:
        drive.flipper_velocity(velocities.data[0], velocities.data[1], velocities.data[2], velocities.data[3])
    except Exception as e:
        debug_publish("ERROR in flipper/velocity: " + str(e))
    rospy.loginfo("Recieved on flipper/velocity: " + ' '.join(map(str, velocities.data)))


def _drive_vel_limits_cb(velocities):
    try:
        drive.set_vel_limits(velocities.data)
    except Exception as e:
        debug_publish("ERROR in drive/velocity_limits: " + str(e))
    rospy.loginfo("Recieved on drive/velocity_limits: " + ' '.join(map(str, velocities.data)))


def _drive_acc_limits_cb(accs):
    try:
        drive.set_acc_limits(accs.data)
    except Exception as e:
        debug_publish("ERROR in drive/velocity_limits: " + str(e))
    rospy.loginfo("Recieved on drive/velocity_limits: " + ' '.join(map(str, accs.data)))


# PUBLISHERS


def debug_publish(messege):
    try:
        msg_str = msg.String(data=messege)
        debug_pub.publish(msg_str)
        rospy.loginfo("Sent on debug: " + str(msg_str.data))
    except rospy.ROSInterruptException:
        pass


def axis_states_publish(axes):
    try:
        msg_arr = msg.Float32MultiArray()
        msg_arr.data = axes
        msg_arr.layout.dim = [msg.MultiArrayDimension(), msg.MultiArrayDimension(), msg.MultiArrayDimension()]
        msg_arr.layout.dim[0].size = len(axes)
        msg_arr.layout.dim[1].size = len(axes[0])
        msg_arr.layout.dim[2].size = len(axes[0][0])
        msg_arr.layout.dim[0].stride = len(axes) * len(axes[0]) * len(axes[0][0])
        msg_arr.layout.dim[1].stride = len(axes[0]) * len(axes[0][0])
        msg_arr.layout.dim[2].stride = len(axes[0][0])
        msg_arr.layout.dim[0].label = "odrives"
        msg_arr.layout.dim[1].label = "axes"
        msg_arr.layout.dim[2].label = "values"

        # print(msg_arr.data)

        # print(msg_arr)

        axis_state_pub.publish(msg_arr)
        # rospy.loginfo("Sent on debug: " + str(axes))
    except rospy.ROSInterruptException:
        pass


# TEST
def _send_back(msg_str):
    debug_publish(msg_str.data)


# FUNCTIONS
def init_handlers():
    rospy.init_node('nuc_main', anonymous=True)

    # SUBSCRIBERS
    rospy.Subscriber("commands", msg.Char, _commands_cb, queue_size=100)
    rospy.Subscriber("drive/distance", msg.Float32MultiArray, _drive_distance_cb, queue_size=1)
    rospy.Subscriber("drive/velocity", msg.Float32MultiArray, _drive_velocity_cb, queue_size=1)
    rospy.Subscriber("drive/vel_limits", msg.Float32MultiArray, _drive_vel_limits_cb, queue_size=1)
    rospy.Subscriber("drive/acc_limits", msg.Float32MultiArray, _drive_acc_limits_cb, queue_size=1)
    rospy.Subscriber("flippers/velocity", msg.Float32MultiArray, _flipper_velocity_cb, queue_size=1)
    rospy.Subscriber("flippers/position", msg.Float32MultiArray, _flippers_position_cb, queue_size=1)

    rospy.Subscriber("send_back", msg.String, _send_back)  # TEST

    global debug_pub
    debug_pub = rospy.Publisher('debug', msg.String, queue_size=100)

    global actual_flipper_pos, actual_drive_pos
    axis_state_pub = rospy.Publisher('drive/axis_states', msg.Float32MultiArray, queue_size=10)


# TODO ?
#   void _drive_limits_cb( const std_msgs::Float32MultiArray& limits){
#     odrive_set_limits(0, limits.data[0], limits.data[1]);
#     digitalWrite(13, HIGH-digitalRead(13));  //toggle led
#   }
#   ros::Subscriber<std_msgs::Float32MultiArray> drive_limits_sub("drive/limits", _drive_limits_cb);
