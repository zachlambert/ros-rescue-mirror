import rospy
from std_msgs.msg import MultiArrayDimension
from rescue_hardware.srv import (
    ReadHardware, ReadHardwareResponse,
    WriteHardware, WriteHardwareResponse
)

# Setup hardware interface

hardware_connected = False
print("Flippers hardware not connected")
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

def flippers_read(req):
    if hardware_connected:
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
    rospy.spin()
