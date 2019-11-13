#!/usr/bin/env python
import odrive
from fibre import Logger, Event
import rospy
import odrive_enums as oenums
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs import msg

shutdown_token = Event()

logger = Logger(verbose=False)

my_odrive = None

diagnostic_pub = rospy.Publisher('odrivediag', DiagnosticStatus, queue_size=10)


def diagnostics():
    global serial_no
    message = DiagnosticStatus()
    message.name = "ODrive report"
    message.message = "ODrive diagnostics"
    message.hardware_id = serial_no
    if my_odrive is None:
        message.level = 1
        message.values = [KeyValue("connected", str(False))]
    else:
        message.values = [
            KeyValue("connected", str(True)),
            KeyValue("bus-voltage", str(my_odrive.vbus_voltage)),
            KeyValue("axis0-error", oenums.errors.axis[my_odrive.axis0.error]),
            KeyValue("axis0-encoder-error", oenums.errors.encoder[my_odrive.axis0.encoder.error]),
            KeyValue("axis0-motor-error", oenums.errors.motor[my_odrive.axis0.motor.error]),
            # KeyValue("axis0-controller-error", oenums.errors.controller[my_odrive.axis0.controller.error]),
            KeyValue("axis0-ctrl-mode", oenums.control_modes[my_odrive.axis0.controller.config.control_mode]),
            KeyValue("axis0-state", oenums.axis_states[my_odrive.axis0.current_state]),
            KeyValue("axis1-error", oenums.errors.axis[my_odrive.axis1.error]),
            KeyValue("axis1-encoder-error", oenums.errors.encoder[my_odrive.axis1.encoder.error]),
            KeyValue("axis1-motor-error", oenums.errors.motor[my_odrive.axis1.motor.error]),
            # KeyValue("axis1-controller-error", oenums.errors.controller[my_odrive.axis1.controller.error]),
            KeyValue("axis1-ctrl-mode", oenums.control_modes[my_odrive.axis1.controller.config.control_mode]),
            KeyValue("axis1-state", oenums.axis_states[my_odrive.axis1.current_state]),
        ]

    # print(repr(message))
    diagnostic_pub.publish(message)


def clear_errors(axis):
    axis.error = 0
    axis.encoder.error = 0
    axis.motor.error = 0
    axis.controller.error = 0


def did_lose_device(device):
    global my_odrive
    print(rospy.get_name() + ": Lost an odrive!")
    my_odrive = None


def did_discover_device(device):
    global my_odrive
    print(rospy.get_name() + ": Found an odrive!")
    device.__channel__._channel_broken.subscribe(lambda: did_lose_device(device))
    my_odrive = device


def vel_setpoint(axis, value):
    print("set " + axis + " to " + str(value))
    if my_odrive:
        axis = my_odrive.axis0 if axis == "axis0" else my_odrive.axis1
        axis.controller.config.control_mode = 2
        axis.controller.vel_setpoint = int(value.data)


rospy.init_node("odrive")
print("Well I've inited the node, gonna listen now")

serial_no = rospy.get_param("~serial_no")

rospy.Subscriber(rospy.get_name() + "/axis0/vel_setpoint", msg.Int32, lambda value: vel_setpoint("axis0", value), queue_size=1) # Need to use ros param server to give this nice values but default to axis0/axis1
rospy.Subscriber(rospy.get_name() + "/axis1/vel_setpoint", msg.Int32, lambda value: vel_setpoint("axis1", value), queue_size=1)

odrive.find_all("usb", serial_no, did_discover_device, shutdown_token, shutdown_token, logger)

r = rospy.Rate(1)
while not rospy.core.is_shutdown():
    r.sleep()
    diagnostics()

