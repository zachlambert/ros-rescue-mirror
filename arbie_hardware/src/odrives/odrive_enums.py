axis_states = {
    0: "AXIS_STATE_UNDEFINED",
    1: "AXIS_STATE_IDLE",
    2: "AXIS_STATE_STARTUP_SEQUENCE",
    3: "AXIS_STATE_FULL_CALIBRATION_SEQUENCE",
    4: "AXIS_STATE_MOTOR_CALIBRATION",
    5: "AXIS_STATE_SENSORLESS_CONTROL",
    6: "AXIS_STATE_ENCODER_INDEX_SEARCH",
    7: "AXIS_STATE_ENCODER_OFFSET_CALIBRATION",
    8: "AXIS_STATE_CLOSED_LOOP_CONTROL",
    9: "AXIS_STATE_LOCKIN_SPIN",
    10: "AXIS_STATE_ENCODER_DIR_FIND"
}


class errors:
    axis = {
        0x00: "ERROR_NONE",
        0x01: "ERROR_INVALID_STATE",
        0x02: "ERROR_DC_BUS_UNDER_VOLTAGE",
        0x04: "ERROR_DC_BUS_OVER_VOLTAGE",
        0x08: "ERROR_CURRENT_MEASUREMENT_TIMEOUT",
        0x10: "ERROR_BRAKE_RESISTOR_DISARMED",
        0x20: "ERROR_MOTOR_DISARMED",
        0x40: "ERROR_MOTOR_FAILED",
        0x80: "ERROR_SENSORLESS_ESTIMATOR_FAILED",
        0x100: "ERROR_ENCODER_FAILED",
        0x200: "ERROR_CONTROLLER_FAILED",
        0x400: "ERROR_POS_CTRL_DURING_SENSORLESS",
        0x800: "ERROR_WATCHDOG_TIMER_EXPIRED"
    }

    motor = {
        0: "ERROR_NONE",
        0x0001: "ERROR_PHASE_RESISTANCE_OUT_OF_RANGE",
        0x0002: "ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE",
        0x0004: "ERROR_ADC_FAILED",
        0x0008: "ERROR_DRV_FAULT",
        0x0010: "ERROR_CONTROL_DEADLINE_MISSED",
        0x0020: "ERROR_NOT_IMPLEMENTED_MOTOR_TYPE",
        0x0040: "ERROR_BRAKE_CURRENT_OUT_OF_RANGE",
        0x0080: "ERROR_MODULATION_MAGNITUDE",
        0x0100: "ERROR_BRAKE_DEADTIME_VIOLATION",
        0x0200: "ERROR_UNEXPECTED_TIMER_CALLBACK",
        0x0400: "ERROR_CURRENT_SENSE_SATURATION",
        0x1000: "ERROR_CURRENT_UNSTABLE"
    }

    encoder = {
        0: "ERROR_NONE",
        0x01: "ERROR_UNSTABLE_GAIN",
        0x02: "ERROR_CPR_OUT_OF_RANGE",
        0x04: "ERROR_NO_RESPONSE",
        0x08: "ERROR_UNSUPPORTED_ENCODER_MODE",
        0x10: "ERROR_ILLEGAL_HALL_STATE",
        0x20: "ERROR_INDEX_NOT_FOUND_YET"
    }

    controller = {
        0: "ERROR_NONE",
        0x01: "ERROR_OVERSPEED",
    }

motor_types = {
    0: "MOTOR_TYPE_HIGH_CURRENT",
    1: "MOTOR_TYPE_LOW_CURRENT",
    2: "MOTOR_TYPE_GIMBAL"
}

control_modes = {
    0: "CTRL_MODE_VOLTAGE_CONTROL",
    1: "CTRL_MODE_CURRENT_CONTROL",
    2: "CTRL_MODE_VELOCITY_CONTROL",
    3: "CTRL_MODE_POSITION_CONTROL",
    4: "CTRL_MODE_TRAJECTORY_CONTROL"
}

encoder_modes = {
    0: "ENCODER_MODE_INCREMENTAL",
    1: "ENCODER_MODE_HALL"
}
