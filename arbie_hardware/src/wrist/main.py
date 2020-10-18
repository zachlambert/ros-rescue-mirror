import rospy
from std_msgs.msg import MultiArrayDimension
from arbie_msgs.srv import (
    ReadHardware, ReadHardwareResponse,
    WriteHardware, WriteHardwareResponse
)
from XM430_control_functions import XM430controller
from AX12_control_functions import AX12controller


# Setup hardware interface

hardware_connected = False

try:
    deviceName = '/dev/ttyUSB0'
    XMid = 2
    AXid = 3
    AXid2 = 5
    XM = XM430controller(deviceName)
    AX = AX12controller(deviceName)

    print("---------------- XM430 ----------------")
    XM.initCommunication()
    XM.setDriveMode(XMid)
    XM.setDriveTime(XMid, 1000)
    XM.enableTorque(XMid)

    print("\n \n---------------- AX12 ----------------")
    AX.initCommunication()
    AX.enableTorque(AXid)
    AX.enableTorque(AXid2)
    print("\n \n")

    AX.writeSpeed(AXid, 400)
    AX.writeSpeed(AXid2, 400)

    hardware_connected = True
except:
    print("Wrist hardware not connected")

def convert_pos_to_hardware(pos1, pos2, pos3):
    # Angle 1
    if pos1<-90:
        pos1 = -90
    if pos1>90:
        pos1 = 90
    hw_pos1 = 2342 - 4096 * pos1/360
    # Angle 2
    if pos2<-90:
        pos2 = -90
    if pos2>90:
        pos2 = 90
    hw_pos2 = 512 - 1024 * pos2/300
    # Angle 3
    if pos3<-150:
        pos3 = -150
    if pos3>150:
        pos3 = 150
    hw_pos3 = 512 + 1024 * pos3/300
    return int(hw_pos1), int(hw_pos2), int(hw_pos3)

def convert_pos_from_hardware(hw_pos1, hw_pos2, hw_pos3):
    # Angle 1
    pos1 = 360 * (2342 - hw_pos1)/4096
    # Angle 2
    pos2 = 300 * (512 - hw_pos2)/1024
    # Angle 3
    pos3 = 300 * (512 - hw_pos3)/1024
    return pos1, pos2, pos3

# Setup ROS interface

dim = MultiArrayDimension(size=3, stride=1, label="joint")
wrist_res = ReadHardwareResponse()
wrist_res.pos.layout.dim.append(dim)
wrist_res.vel.layout.dim.append(dim)
wrist_res.eff.layout.dim.append(dim)
wrist_res.pos.data = [0, 0, 0]
wrist_res.vel.data = [0, 0, 0]
wrist_res.eff.data = [0, 0, 0]
wrist_res.success = True

def wrist_read(req):
    if hardware_connected:
        hw_pos1 = XM.readPosition(XMid)
        hw_pos2 = AX.readPosition(AXid)
        hw_pos3 = AX.readPosition(AXid2)
        req.pos.data = convert_pos_from_hardware(hw_pos1, hw_pos2, hw_pos3)
    return wrist_res

def wrist_write(req):
    if hardware_connected:
        hw_pos = convert_pos_to_hardware(*req.pos.data)
        XM.writePosition(XMid, hw_pos[0])
        AX.writePosition(AXid, hw_pos[1])
        AX.writePosition(AXid2, hw_pos[2])
    else:
        wrist_res.pos.data = req.cmd.data
    return WriteHardwareResponse(True)

def main():
    rospy.init_node("wrist")
    rospy.Service("wrist/read", ReadHardware, wrist_read)
    rospy.Service("wrist/write", WriteHardware, wrist_write)
    rospy.spin()
