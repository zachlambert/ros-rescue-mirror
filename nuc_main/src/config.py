DRIVE = {
    "WHEEL_RADIUS": 0.07,
    "DRIVE_GEARING": 26 * 3,  # 64 means a gearing of 64 to one
    "FLIPPER_GEARING": 16,
    "TRACKS_SEPARATION": 0.3,
    "CPR": 8192,
    "MAX_DRIVE_SPEED": 1000,
    "MAX_FLIPPER_SPEED": 1000,
    "MAX_CURRENT": 20
}

ODRIVES = {
    "FLIPPER": {
        "SERIAL_NO": "336631563536",
        "SERIAL_PORT": "serial:/dev/ttyACM0",
        "FRONT": {
            "AXIS": 0,
            "DIRECTION": 1
        },
        "REAR": {
            "AXIS": 1,
            "DIRECTION": 1
        }
    },
    "DRIVE": {
        "SERIAL_NO": "209137933548",
        "SERIAL_PORT": "serial:/dev/ttyACM1",
        "LEFT": {
            "AXIS": 0,
            "DIRECTION": 1
        },
        "RIGHT": {
            "AXIS": 1,
            "DIRECTION": 1
        }
    }
}

AXIS_LOG = {
    "PERIOD": 5,
    "ID_STATE": 0,
    "ID_VEL": 1,
    "ID_VEL_SET": 2,
    "ID_POS_SET": 3,
    "ID_POS": 4
}

AXIS_STATES_ENUM = {
    'AXIS_STATE_UNDEFINED': 0,
    'AXIS_STATE_IDLE': 1,
    'AXIS_STATE_STARTUP_SEQUENCE': 2,
    'AXIS_STATE_FULL_CALIBRATION_SEQUENCE': 3,
    'AXIS_STATE_MOTOR_CALIBRATION': 4,
    'AXIS_STATE_SENSORLESS_CONTROL': 5,
    'AXIS_STATE_ENCODER_INDEX_SEARCH': 6,
    'AXIS_STATE_ENCODER_OFFSET_CALIBRATION': 7,
    'AXIS_STATE_CLOSED_LOOP_CONTROL': 8,
    'AXIS_STATE_LOCKIN_SPIN': 9,
    'AXIS_STATE_ENCODER_DIR_FIND': 10,
}

CTRL_MODES_ENUM = {
    'CTRL_MODE_VOLTAGE_CONTROL': 0,
    'CTRL_MODE_CURRENT_CONTROL': 1,
    'CTRL_MODE_VELOCITY_CONTROL': 2,
    'CTRL_MODE_POSITION_CONTROL': 3,
    'CTRL_MODE_TRAJECTORY_CONTROL': 4
}
