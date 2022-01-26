
from niryo_robot_status.msg import RobotStatus
from rosgraph_msgs.msg import Log

LOG_LEVEL_TO_STR = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}

LOG_LEVEL_TO_MSG = {
    Log.DEBUG: RobotStatus.NONE,
    Log.INFO: RobotStatus.NONE,
    Log.WARN: RobotStatus.WARN,
    Log.ERROR: RobotStatus.ERROR,
    Log.FATAL: RobotStatus.FATAL,
}

ROBOT_STATUS_TO_STR = {
    RobotStatus.UPDATE: 'Update',
    RobotStatus.REBOOT: 'Reboot',
    RobotStatus.SHUTDOWN: 'Shutdown',
    RobotStatus.FATAL_ERROR: 'Fatal error',
    RobotStatus.MOTOR_ERROR: 'Motor Error',
    RobotStatus.COLLISION: 'Collision',
    RobotStatus.USER_PROGRAM_ERROR: 'User program error',
    RobotStatus.UNKNOWN: 'Unknown',
    RobotStatus.BOOTING: 'Booting',
    RobotStatus.CALIBRATION_NEEDED: 'Calibration Needed',
    RobotStatus.CALIBRATION_IN_PROGRESS: 'Calibration in progress',
    RobotStatus.LEARNING_MODE: 'Learning mode',
    RobotStatus.STANDBY: 'Standby',
    RobotStatus.MOVING: 'Moving',
    RobotStatus.RUNNING_AUTONOMOUS: 'Running autonomous',
    RobotStatus.RUNNING_DEBUG: 'Running debug',
    RobotStatus.PAUSE: 'Pause',
    RobotStatus.LEARNING_MODE_AUTONOMOUS: 'Learning mode autonomous',
    RobotStatus.REBOOT_MOTOR: 'Reboot motor'
}

LOG_STATUS_TO_STR = {
    RobotStatus.FATAL: 'Fatal',
    RobotStatus.ERROR: 'Error',
    RobotStatus.WARN: 'Warn',
    RobotStatus.NONE: 'None',
}
