#!/usr/bin/env python

from niryo_robot_led_ring.msg import LedRingStatus, LedRingAnimation, LedRingCurrentState

# Message
from niryo_robot_status.msg import RobotStatus
from std_msgs.msg import ColorRGBA

RED = ColorRGBA(255, 0, 0, 0)
GREEN = ColorRGBA(50, 255, 0, 0)
BLUE = ColorRGBA(15, 50, 255, 0)
WHITE = ColorRGBA(255, 255, 255, 0)
NONE = ColorRGBA(0, 0, 0, 0)
ORANGE = ColorRGBA(255, 50, 0, 0)

ROBOT_STATUS_TO_ANIM = {
    RobotStatus.SHUTDOWN: [LedRingAnimation.NONE, NONE],
    RobotStatus.FATAL_ERROR: [LedRingAnimation.SOLID, RED],
    RobotStatus.MOTOR_ERROR: [LedRingAnimation.FLASHING, RED],
    RobotStatus.COLLISION: [LedRingAnimation.FLASHING, ORANGE],
    RobotStatus.USER_PROGRAM_ERROR: [LedRingAnimation.BREATH, ORANGE],
    RobotStatus.UNKNOWN: [LedRingAnimation.NONE, NONE],
    RobotStatus.BOOTING: [LedRingAnimation.BREATH, WHITE],
    RobotStatus.UPDATE: [LedRingAnimation.SOLID, WHITE],
    RobotStatus.CALIBRATION_NEEDED: [LedRingAnimation.CHASE, BLUE],
    RobotStatus.CALIBRATION_IN_PROGRESS: [LedRingAnimation.SNAKE, BLUE],
    RobotStatus.LEARNING_MODE: [LedRingAnimation.BREATH, BLUE],
    RobotStatus.STANDBY: [LedRingAnimation.BREATH, GREEN],
    RobotStatus.MOVING: [LedRingAnimation.BREATH, GREEN],
    RobotStatus.RUNNING_AUTONOMOUS: [LedRingAnimation.SOLID, GREEN],
    RobotStatus.RUNNING_DEBUG: [LedRingAnimation.SOLID, GREEN],
    RobotStatus.PAUSE: [LedRingAnimation.CHASE, GREEN],
    RobotStatus.LEARNING_MODE_AUTONOMOUS: [LedRingAnimation.SOLID, GREEN],
}

ROBOT_ANIM_TO_STRING = {
    LedRingAnimation.NONE: "None",
    LedRingAnimation.SOLID: "Solid",
    LedRingAnimation.FLASHING: "Flashing",
    LedRingAnimation.ALTERNATE: "Alternate",
    LedRingAnimation.CHASE: "Chase",
    LedRingAnimation.COLOR_WIPE: "Color wipe",
    LedRingAnimation.RAINBOW: "Rainbow",
    LedRingAnimation.RAINBOW_CYLE: "Rainbow cycle",
    LedRingAnimation.RAINBOW_CHASE: "Rainbow chase",
    LedRingAnimation.GO_UP: "Go up",
    LedRingAnimation.GO_UP_AND_DOWN: "Go up and down",
    LedRingAnimation.BREATH: "Breath",
    LedRingAnimation.SNAKE: "Snake",
}
