import rospy
from std_msgs.msg import MultiArrayDimension
from rescue_hardware.srv import (
    ReadHardware, ReadHardwareResponse,
    WriteHardware, WriteHardwareResponse
)
from odrive_functions import OdriveController

# Setup hardware interface

# TODO: flippers = OdriveController(rospy.get_param("~flippers_serial_no")
flippers = OdriveController("336631563536")

# TODO: flippers = OdriveController(rospy.get_param("~flippers_serial_no")
flippers = OdriveController("209137933548")

if not flippers.is_connected():
    print("Flippers hardware not connected")

if not flippers.is_connected():
    print("Flippers hardware not connected")


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

def flippers_read(req):
    if flippers.is_connected():
        pass # TODO
    flippers_res.success = True
    return flippers_res

def flippers_write(req):
    if hardware_connected:
        pass # TODO
    else:
        flippers_res.vel.data = req.cmd.data
        for i in range(len(flippers_res.pos.data)):
            flippers_res.pos.data[i] += flippers_res.vel.data[i] * 0.1 # Arbitrary fake dt
    return WriteHardwareResponse(True)

def tracks_read(req):
    if hardware_connected:
        pass # TODO
    tracks_res.success = True
    return tracks_res

def tracks_write(req):
    if hardware_connected:
        pass #TODO
    else:
        tracks_res.vel.data = req.cmd.data
        for i in range(len(tracks_res.pos.data)):
            tracks_res.pos.data[i] += tracks_res.vel.data[i] * 0.1 # Arbitrary fake dt
    return WriteHardwareResponse(True)

def main():
    rospy.init_node("odrives")
    rospy.Service("flippers/read", ReadHardware, flippers_read)
    rospy.Service("flippers/write", WriteHardware, flippers_write)
    rospy.Service("tracks/read", ReadHardware, tracks_read)
    rospy.Service("tracks/write", WriteHardware, tracks_write)

    flippers_diagnostic_pub = rospy.Publisher(
        "flippers/diagnostic", DiagnosticStatus, 10)
    tracks_diagnostic_pub = rospy.Publisher(
        "flippers/diagnostic", DiagnosticStatus, 10)

    rate = rospy.Rate(1);
    while not rospy.is_shutdown():
        flippers_diagnostic_pub.publish(flippers.get_diagnostics_message);
        tracks_diagnostic_pub.publish(tracks.get_diagnostics_message);
        rate.sleep()
