# example code for using the XM430 motors

from XM430_control_functions import XM430controller
import time

deviceName = '/dev/tty.usbserial-FT3M4HIL'
XMid = 2
waitTime = 4

XM = XM430controller(deviceName)

XM.initCommunication()
XM.setDriveMode(XMid)
XM.setDriveTime(XMid, 3000)

XM.enableTorque(XMid)


XM.writePosition(XMid, 2000)
time.sleep(waitTime)
XM.writePosition(XMid, 0)

while 1:
	XM.readPosition(XMid)

	time.sleep(0.5)
