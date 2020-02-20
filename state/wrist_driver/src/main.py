#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import Float32MultiArray

from XM430_control_functions import XM430controller
from AX12_control_functions import AX12controller


# Setup dynamixel control

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

def convert_angles(angle1, angle2, angle3):
    # Angle 1
    if angle1<-90:
        angle1 = -90
    if angle1>90:
        angle1 = 90
    pos1 = 2342 - (4096/360)*angle1
    # Angle 2
    if angle2<-90:
        angle2 = -90
    if angle2>90:
        angle2 = 90
    pos2 = 512 - (1024/300)*angle2
    # Angle 3
    if angle3<-150:
        angle3 = -150
    if angle3>150:
        angle3 = 150
    pos3 = 512 + (1024/300)*angle3
    return int(pos1), int(pos2), int(pos3)

def callback(msg):
    pos1, pos2, pos3 = convert_angles(*msg.data)
    # Convert angles to correct positions
    # Control each of the 3 dynamixels
    XM.writePosition(XMid, pos1)
    AX.writePosition(AXid, pos3)
    AX.writePosition(AXid2, pos2)

rospy.init_node('wrist_driver')
wrist_angles_sub = rospy.Subscriber('/wrist_demand_angles', Float32MultiArray, callback)
rospy.spin()





