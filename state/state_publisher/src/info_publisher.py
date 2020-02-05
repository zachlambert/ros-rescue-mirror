import rospy
import tf

from geometry_msgs.msg import Point

def init():
    global tf_listener, actuator_pos_pub
    tf_listener = tf.TransformListener()
    actuator_pos_pub = rospy.Publisher('/gripper/position', Point, queue_size=1)


def publish():
    try:
        (trans, _) = tf_listener.lookupTransform('/world', '/gripper_1', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    point = Point(*trans)
    actuator_pos_pub.publish(point)
