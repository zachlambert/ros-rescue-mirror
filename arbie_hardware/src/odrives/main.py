import rospy
from std_msgs.msg import MultiArrayDimension, Float32
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rescue_hardware.srv import (
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

dim = MultiArrayDimension(size=2, stride=1, label="joint")

flippers_res = ReadHardwareResponse()
flippers_res.pos.layout.dim.append(dim)
flippers_res.vel.layout.dim.append(dim)
flippers_res.eff.layout.dim.append(dim)
flippers_res.pos.data = [0, 0]
flippers_res.vel.data = [0, 0]
flippers_res.eff.data = [0, 0]

tracks_res = ReadHardwareResponse()
tracks_res.pos.layout.dim.append(dim)
tracks_res.vel.layout.dim.append(dim)
tracks_res.eff.layout.dim.append(dim)
tracks_res.pos.data = [0, 0]
tracks_res.vel.data = [0, 0]
tracks_res.eff.data = [0, 0]

# TODO: Set these as parameters
flippers_front_offset = 373
flippers_rear_offset =  605
flippers_scale = 2 * pi/1024.0

def set_flippers_front_position(raw_value):
    flippers_res.pos.data[0] = (raw_value - flippers_front_offset) * flippers_scale

def set_flippers_rear_position(raw_value):
    flippers_res.pos.data[1] = (raw_value - flippers_rear_offset) * flippers_scale

def flippers_read(req):
    if flippers.is_connected():
        flippers_res.success = True
    else:
        flipper_res.success = False
    return flippers_res

def flippers_write(req):
    if flippers.is_connected():
        flippers.write_velocity_axis0(flippers_res.vel.data[0])
        flippers.write_velocity_axis1(flippers_res.vel.data[1])
        flippers_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)

def tracks_read(req):
    if tracks.is_connected():
        tracks_res.success = True
    else:
        tracks_res.success = False
    return tracks_res

def tracks_write(req):
    if tracks.is_connected():
        tracks.write_velocity_axis0(flippers_res.vel.data[0])
        flippers.write_velocity_axis1(flippers_res.vel.data[1])
        tracks_res.vel.data = req.cmd.data
        return WriteHardwareResponse(True)
    else:
        return WriteHardwareResponse(False)
    return WriteHardwareResponse(True)

def main():
    rospy.init_node("odrives")
    rospy.Service("flippers/read", ReadHardware, flippers_read)
    rospy.Service("flippers/write", WriteHardware, flippers_write)
    rospy.Service("tracks/read", ReadHardware, tracks_read)
    rospy.Service("tracks/write", WriteHardware, tracks_write)

    rospy.Subscriber("/flipper/front", Float32, set_flippers_front_position)
    rospy.Subscriber("/flipper/rear", Float32, set_flippers_rear_position)

    flippers_diagnostic_pub = rospy.Publisher(
        "flippers/diagnostic", DiagnosticStatus, 10)
    tracks_diagnostic_pub = rospy.Publisher(
        "flippers/diagnostic", DiagnosticStatus, 10)

    rate = rospy.Rate(1);
    while not rospy.is_shutdown():
        flippers_diagnostic_pub.publish(flippers.get_diagnostics_message());
        tracks_diagnostic_pub.publish(tracks.get_diagnostics_message());
        rate.sleep()
