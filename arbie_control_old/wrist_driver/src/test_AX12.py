# example code for using the AX12 motors

from AX12_control_functions import AX12controller
import time

deviceName = '/dev/tty.usbserial-FT3M4HIL'
AXid = 3
waitTime = 2

AX = AX12controller(deviceName)

AX.initCommunication()
AX.ping(AXid)
AX.enableTorque(AXid)

AX.writeSpeed(AXid, 100)
AX.writePosition(AXid, 1000)
time.sleep(waitTime)
AX.writePosition(AXid, 0)
time.sleep(waitTime)