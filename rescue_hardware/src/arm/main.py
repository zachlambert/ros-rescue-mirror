#!/usr/bin/python
import rospy
from std_msgs.msg import MultiArrayDimension
from rescue_hardware.srv import (
    ReadHardware, ReadHardwareResponse,
    WriteHardware, WriteHardwareResponse
)

# Setup hardware interface

hardware_connected = False
print("Arm hardware not connected")

# Setup ROS interface

dim = MultiArrayDimension(size=3, stride=1, label="joint")
arm_res = ReadHardwareResponse()
arm_res.pos.layout.dim.append(dim)
arm_res.vel.layout.dim.append(dim)
arm_res.eff.layout.dim.append(dim)
arm_res.pos.data = [0, 0, 0]
arm_res.vel.data = [0, 0, 0]
arm_res.eff.data = [0, 0, 0]

def arm_read(req):
    if hardware_connected:
        pass # TODO
    arm_res.success = True
    return arm_res

def arm_write(req):
    if hardware_connected:
        pass # TODO
    else:
        arm_res.pos.data = req.cmd.data
    return WriteHardwareResponse(True)

def main():
    rospy.init_node("arm")
    rospy.Service("arm/read", ReadHardware, arm_read)
    rospy.Service("arm/write", WriteHardware, arm_write)
    rospy.spin()
