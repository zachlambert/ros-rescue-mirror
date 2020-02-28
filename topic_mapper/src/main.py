#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray

rospy.init_node("topic_mapper")

flipper = {"front": 512.0, "rear": 512.0}
offset = {"front": 373.0, "rear": 605.0}

scale = 2 * 3.1415/1024.0


def receive_flipper_data(message, side):
    global pub_flipper, flipper
    flipper[side] = (message.data - offset[side]) * scale

    msg = Float32MultiArray()

    msg.data = [flipper["front"], flipper["rear"]]

    pub_flipper.publish(msg)


rospy.Subscriber("/flipper/front", Float32, lambda msg: receive_flipper_data(msg, "front"))
rospy.Subscriber("/flipper/rear", Float32, lambda msg: receive_flipper_data(msg, "rear"))

pub_flipper = rospy.Publisher('/flippers/position', Float32MultiArray, queue_size=1)

rospy.spin()