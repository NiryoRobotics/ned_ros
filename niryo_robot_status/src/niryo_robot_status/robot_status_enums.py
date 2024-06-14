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
    RobotStatus.LEARNING_TRAJECTORY: 'Executing Trajectory',
    RobotStatus.REBOOT_MOTOR: 'Reboot motor',
    RobotStatus.ESTOP: 'Emergency stop'
}

ROBOT_STATUS_TO_MESSAGE = {
    RobotStatus.UPDATE: "Update",
    RobotStatus.REBOOT: "Rebooting one or more motors",
    RobotStatus.SHUTDOWN: 'Shutdown',
    RobotStatus.COLLISION: "Robot collision detected, waiting for user intervention",
    RobotStatus.USER_PROGRAM_ERROR: 'User program error',
    RobotStatus.BOOTING: "Robot is booting",
    RobotStatus.CALIBRATION_NEEDED: 'Calibration Needed',
    RobotStatus.CALIBRATION_IN_PROGRESS: "Calibration in progress",
    RobotStatus.LEARNING_MODE: "Learning mode activated",
    RobotStatus.STANDBY: "Standby, nothing else to say",
    RobotStatus.MOVING: "Robot is moving",
    RobotStatus.RUNNING_AUTONOMOUS: "Program is running",
    RobotStatus.RUNNING_DEBUG: "Debug program is running",
    RobotStatus.PAUSE: "Program paused",
    RobotStatus.LEARNING_TRAJECTORY: "User is leaning the arm a trajectory",
    RobotStatus.LEARNING_MODE_AUTONOMOUS: "Program is running and learning_mode active",
    RobotStatus.REBOOT_MOTOR: 'Reboot motor',
    RobotStatus.ESTOP: 'Emergency stop'
}

LOG_STATUS_TO_STR = {
    RobotStatus.FATAL: 'Fatal',
    RobotStatus.ERROR: 'Error',
    RobotStatus.WARN: 'Warn',
    RobotStatus.NONE: 'None',
}
