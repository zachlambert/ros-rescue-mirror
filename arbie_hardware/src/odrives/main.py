import rospy
from std_msgs.msg import MultiArrayDimension, Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from arbie_msgs.srv import (
    ReadHardware, ReadHardwareResponse,
    WriteHardware, WriteHardwareResponse
)
from odrive_controller import OdriveController
from math import pi

# Setup hardware interface

# TODO: flippers = OdriveController(rospy.get_param("~flippers_serial_no")
flippers = OdriveController("336631563536")

# TODO: flippers = OdriveController(rospy.get_param("~flippers_serial_no")
tracks = OdriveController("209137933548")

if not flippers.is_connected():
    print("Flippers hardware not connected")

if not tracks.is_connected():
    print("Tracks hardware not connected")


# Setup ROS interface

flippers_front_res = ReadHardwareResponse()
flippers_rear_res = ReadHardwareResponse()

tracks_front_res = ReadHardwareResponse()
tracks_rear_res = ReadHardwareResponse()

# TODO: Set these as parameters
flippers_front_offset = 373
flippers_rear_offset =  605
flippers_scale = 2 * pi/1024.0


# Service callbacks for flippers_front_...

def flippers_front_read(req):
    flippers_front_res.success = flippers.is_connected()
    return flippers_front_res

def flippers_front_write(req):
    if flippers.is_connected():
        # WRITE AXIS0
        flippers.write_velocity_axis0(req.cmd.data)
        flippers_front_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)


# Service callbacks for flippers_rear_...

def flippers_rear_read(req):
    flippers_rear_res.success = flippers.is_connected()
    return flippers_rear_res

def flippers_rear_write(req):
    if flippers.is_connected():
        # WRITE AXIS1
        flippers.write_velocity_axis1(req.cmd.data)
        flippers_front_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)


# Service callbacks for tracks_front_...

def tracks_front_read(req):
    tracks_front_res.success = tracks.is_connected()
    return tracks_front_res

def tracks_front_write(req):
    if tracks.is_connected():
        # WRITE AXIS0
        tracks.write_velocity_axis0(req.cmd.data)
        tracks_front_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)
    return WriteHardwareResponse(True)


# Service callbacks for tracks_front_...

def tracks_rear_read(req):
    tracks_rear_res.success = tracks.is_connected()
    return tracks_rear_res

def tracks_rear_write(req):
    if tracks.is_connected():
        # WRITE AXIS1
        tracks.write_velocity_axis1(req.cmd.data)
        tracks_rear_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)
    return WriteHardwareResponse(True)


# Topic callback from the topics which monitor the flipper angles

def set_flippers_front_position(raw_value):
    flippers_front_res = (raw_value - flippers_front_offset) * flippers_scale

def set_flippers_rear_position(raw_value):
    flippers_rear_res = (raw_value - flippers_rear_offset) * flippers_scale


def main():
    rospy.init_node("odrives")

    # Used by ros_control to read and write joints
    rospy.Service("flippers/front/read", ReadHardware, flippers_front_read)
    rospy.Service("flippers/front/write", WriteHardware, flippers_front_write)
    rospy.Service("flippers/rear/read", ReadHardware, flippers_front_read)
    rospy.Service("flippers/rear/write", WriteHardware, flippers_front_write)

    # Used by ros_control to read and write joints
    rospy.Service("tracks/front/read", ReadHardware, tracks_front_read)
    rospy.Service("tracks/front/write", WriteHardware, tracks_front_write)
    rospy.Service("tracks/rear/read", ReadHardware, tracks_rear_read)
    rospy.Service("tracks/rear/write", WriteHardware, tracks_rear_write)

    # Provides feedback for flipper positions, which is written
    # to the ReadHardware response messages
    rospy.Subscriber("/flippers/front", Float32, set_flippers_front_position)
    rospy.Subscriber("/flippers/rear", Float32, set_flippers_rear_position)

    flippers_diagnostic_pub = rospy.Publisher(
        "flippers/diagnostic", DiagnosticStatus, 10)
    tracks_diagnostic_pub = rospy.Publisher(
        "tracks/diagnostic", DiagnosticStatus, 10)

    rate = rospy.Rate(1);
    while not rospy.is_shutdown():
        flippers_diagnostic_pub.publish(flippers.get_diagnostics_message());
        tracks_diagnostic_pub.publish(tracks.get_diagnostics_message());
        rate.sleep()
