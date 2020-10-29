import odrive
from fibre import Logger, Event
import rospy
import odrive_enums as oenums
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs import msg
from std_srvs.srv import Trigger, TriggerResponse

class OdriveController:

    def __init__(self, name, serial_no):
        self.name = name
        self.serial_no = serial_no
        self.shutdown_token = Event()
        self.logger = Logger(verbose=False)
        self.device = None
        odrive.find_all(
            "usb",
            self.serial_no,
            lambda device: self.did_discover_device(device),
            self.shutdown_token,
            self.shutdown_token,
            self.logger
        )

    def is_connected(self):
        return self.device is not None

    def _calibrate(self, axis):
        if self.device:
            axis.requested_state = 3
            while axis.current_state != 1:  # Wait for calibration to be done
                rospy.sleep(0.1)
            axis.requested_state = 8
            if axis.current_state == 8:
                return TriggerResponse(success=True, message="Calibrated")
            else:
                return TriggerResponse(success=False, message="Unable to go into closed loop control")
        return TriggerResponse(success=False, message="Could not find odrive")

    def calibrate_axis0(self):
        return self._calibrate(self.device.axis0)

    def calibrate_axis1(self):
        return self._calibrate(self.device.axis1)

    def did_discover_device(self, device):
        print(self.name + ": Found an odrive!")
        self.device = device
        device.__channel__._channel_broken.subscribe(lambda: self.did_lose_device())
        my_odrive = device
        my_odrive.axis0.controller.config.vel_limit = 8192 * 40
        my_odrive.axis1.controller.config.vel_limit = 8192 * 40
        my_odrive.axis0.motor.config.current_lim = 30
        my_odrive.axis1.motor.config.current_lim = 30

    def did_lose_device(self):
        print(self.name + ": Lost an odrive!")
        self.device = None

    def get_diagnostics_message(self):
        message = DiagnosticStatus()
        message.name = "ODrive report"
        message.message = "ODrive diagnostics"
        message.hardware_id = self.serial_no
        if self.device is None:
            message.level = 1
            message.values = [KeyValue("connected", str(False))]
        else:
            message.values = [
                KeyValue("connected", str(True)),
                KeyValue("bus-voltage", str(self.device.vbus_voltage)),
                KeyValue("axis0-error", oenums.errors.axis[self.device_odrive.axis0.error]),
                KeyValue("axis0-encoder-error", oenums.errors.encoder[self.device.axis0.encoder.error]),
                KeyValue("axis0-motor-error", oenums.errors.motor[self.device.axis0.motor.error]),
                # KeyValue("axis0-controller-error", oenums.errors.controller[self.device.axis0.controller.error]),
                KeyValue("axis0-ctrl-mode", oenums.control_modes[self.device.axis0.controller.config.control_mode]),
                KeyValue("axis0-state", oenums.axis_states[self.device.axis0.current_state]),
                KeyValue("axis1-error", oenums.errors.axis[self.device.axis1.error]),
                KeyValue("axis1-encoder-error", oenums.errors.encoder[self.device.axis1.encoder.error]),
                KeyValue("axis1-motor-error", oenums.errors.motor[self.device.axis1.motor.error]),
                # KeyValue("axis1-controller-error", oenums.errors.controller[self.device.axis1.controller.error]),
                KeyValue("axis1-ctrl-mode", oenums.control_modes[self.device.axis1.controller.config.control_mode]),
                KeyValue("axis1-state", oenums.axis_states[self.device.axis1.current_state]),
            ]

        # print(repr(message))
        return message

    def _write_velocity(self, value, axis):
        if self.device:
            axis.controller.config.control_mode = 2
            axis.controller.vel_setpoint = int(value)

    def write_velocity_axis0(self, value):
        self._write_velocity(value, self.device.axis0)

    def write_velocity_axis1(self, value):
        self._write_velocity(value, self.device.axis1)
