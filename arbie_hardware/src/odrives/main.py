import rospy
from std_msgs.msg import MultiArrayDimension, Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger, TriggerResponse
from arbie_msgs.srv import (
    ReadHardware, ReadHardwareResponse,
    WriteHardware, WriteHardwareResponse
)
from odrive_controller import OdriveController
from math import pi

# Setup hardware interface

# TODO: flippers = OdriveController(rospy.get_param("~flippers_serial_no")
flippers = OdriveController("flippers", "336631563123")

# TODO: flippers = OdriveController(rospy.get_param("~flippers_serial_no")
tracks = OdriveController("tracks", "336631563536")

if not flippers.is_connected():
    print("Flippers hardware not connected")

if not tracks.is_connected():
    print("Tracks hardware not connected")


# Setup ROS interface

flippers_front_res = ReadHardwareResponse()
flippers_rear_res = ReadHardwareResponse()

tracks_left_res = ReadHardwareResponse()
tracks_right_res = ReadHardwareResponse()

# TODO: Set these as parameters
flippers_front_offset = 373
flippers_rear_offset =  605
flippers_scale = 2 * pi/1024.0

# TODO: Set these as parameters
odrives_cpr = 8192.0
tracks_gear_ratio = 32.0

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

def flippers_front_calibrate(req):
    if flippers.is_connected():
        return flippers.calibrate_axis0()
    else:
        return TriggerResponse(False, "Flippers not connected")

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

def flippers_rear_calibrate(req):
    if flippers.is_connected():
        return flippers.calibrate_axis1()
    else:
        return TriggerResponse(False, "Flippers not connected")

# Service callbacks for tracks_left_...

def tracks_left_read(req):
    tracks_left_res.success = tracks.is_connected()
    return tracks_left_res

def tracks_left_write(req):
    if tracks.is_connected():
        # WRITE AXIS0
        vel_cmd = -(req.cmd.data/(2*pi))*odrives_cpr*tracks_gear_ratio
        tracks.write_velocity_axis0(vel_cmd);
        tracks_left_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)
    return WriteHardwareResponse(True)

def tracks_left_calibrate(req):
    if tracks.is_connected():
        return tracks.calibrate_axis0()
    else:
        return TriggerResponse(False, "Tracks not connected")

# Service callbacks for tracks_left_...

def tracks_right_read(req):
    tracks_right_res.success = tracks.is_connected()
    return tracks_right_res

def tracks_right_write(req):
    if tracks.is_connected():
        # WRITE AXIS1
        vel_cmd = (req.cmd.data/(2*pi))*odrives_cpr*tracks_gear_ratio
        tracks.write_velocity_axis1(vel_cmd)
        tracks_right_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)
    return WriteHardwareResponse(True)

def tracks_right_calibrate(req):
    if tracks.is_connected():
        return tracks.calibrate_axis1()
    else:
        return TriggerResponse(False, "Tracks not connected")

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
    rospy.Service("tracks/left/read", ReadHardware, tracks_left_read)
    rospy.Service("tracks/left/write", WriteHardware, tracks_left_write)
    rospy.Service("tracks/right/read", ReadHardware, tracks_right_read)
    rospy.Service("tracks/right/write", WriteHardware, tracks_right_write)

    # Provides feedback for flipper positions, which is written
    # to the ReadHardware response messages
    rospy.Subscriber("/flippers/front", Float32, set_flippers_front_position)
    rospy.Subscriber("/flippers/rear", Float32, set_flippers_rear_position)

    flippers_diagnostic_pub = rospy.Publisher(
        "flippers/diagnostic", DiagnosticStatus, queue_size=10)
    tracks_diagnostic_pub = rospy.Publisher(
        "tracks/diagnostic", DiagnosticStatus, queue_size=10)

    rospy.Service("flippers/front/calibrate", Trigger, flippers_front_calibrate)
    rospy.Service("flippers/rear/calibrate", Trigger, flippers_rear_calibrate)
    rospy.Service("tracks/left/calibrate", Trigger, tracks_left_calibrate)
    rospy.Service("tracks/right/calibrate", Trigger, tracks_right_calibrate)

    rate = rospy.Rate(1);
    while not rospy.is_shutdown():
        flippers_diagnostic_pub.publish(flippers.get_diagnostics_message());
        tracks_diagnostic_pub.publish(tracks.get_diagnostics_message());
        rate.sleep()
