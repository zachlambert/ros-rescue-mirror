import rospy
import tf

from geometry_msgs.msg import Point

def init():
    global tf_listener, actuator_pos_pub
    tf_listener = tf.TransformListener()
    actuator_pos_pub = rospy.Publisher('/end_effector/position', Point, queue_size=1)


def publish():
    try:
        (trans, _) = tf_listener.lookupTransform('/world', '/end_effector_ref', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    point = Point(*trans)
    actuator_pos_pub.publish(point)
