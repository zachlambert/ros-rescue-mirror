import odrive
from fibre import Logger, Event
import rospy

shutdown_token = Event()

logger = Logger(verbose=False)

def did_lose_device(device):
    print("Lost an odrive!")

def did_discover_device(device):
    print("Found an odrive!")
    device.__channel__._channel_broken.subscribe(lambda: did_lose_device(device))
    # print(repr(obj))

rospy.init_node("odrive")

serial_no = "336631563536"
odrive.find_all("usb", serial_no, did_discover_device, shutdown_token, shutdown_token, logger)

r = rospy.Rate(1)
while not rospy.core.is_shutdown():
   r.sleep()

