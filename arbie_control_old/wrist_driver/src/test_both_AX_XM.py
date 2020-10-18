#!/usr/bin/env python
# example code for using the XM430 and AX12 motors simoultaneously

from XM430_control_functions import XM430controller
from AX12_control_functions import AX12controller
import time

deviceName = '/dev/ttyUSB0'
XMid = 2
AXid = 3
waitTime = 3

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

print("\n \n")


AX.writeSpeed(AXid, 400)
AX.writePosition(AXid, 1000)
XM.writePosition(XMid, 2000)
time.sleep(waitTime)
XM.writePosition(XMid, 0)
AX.writePosition(AXid, 0)
time.sleep(waitTime)

