from __future__ import division

# what topic and values are called

# topics:
# drive/velocity [velociy, ang_velocity] positive is forward, left
# drive/distance [distance, ang_distance] positive is forward, left
# commands char(command) options: 'r' - reset and calibrate all, 's' - stop all
# flippers/velocity [ang_vel_0, ang_vel_1, ang_vel_2, ang_vel_3] positive is up
# flippers/position [ang_pos_LB, ang_pos_LF, ang_pos_RF, ang_pos_RB] positive is up

MAX_DRIVE_SPEED = 0.23
MAX_ANGULAR_SPEED = 1.3
MAX_FLIPPER_SPEED = 2.55

MESSAGES = {
    "drive_vel": (lambda data: ("drive/velocity", (data.get("ABS_RY", 0) / 32768 * MAX_DRIVE_SPEED,
                                                   data.get("ABS_RX", 0) / 32768 * MAX_ANGULAR_SPEED))),
    "odrives_reset": (lambda data: ("commands", 'r' if data.get("BTN_START", 0) == 1 else 0)),  # resets all odrives
    "odrives_stop": (lambda data: ("commands", 's' if data.get("BTN_SELECT", 0) == 1 else 0)),  # stops all odrives
    "flip_vel": (lambda data: ("flippers/velocity", (data.get("ABS_HAT0X", 0) * MAX_FLIPPER_SPEED,
                                                     data.get("ABS_HAT0Y", 0) * MAX_FLIPPER_SPEED,
                                                     data.get("ABS_Y", 0) / 32768 * MAX_FLIPPER_SPEED,
                                                     data.get("ABS_X", 0) / 32768 * MAX_FLIPPER_SPEED)))
}

TRIGGERS = {
    "ABS_RX": "drive_vel",
    "ABS_RY": "drive_vel",
    "BTN_START": "odrives_reset",
    "ABS_HAT0X": "flip_vel",
    "ABS_HAT0Y": "flip_vel",
    "ABS_X": "flip_vel",
    "ABS_Y": "flip_vel",
    "BTN_SELECT": "odrives_stop"
}