#!/usr/bin/env python


class MoveCommandType(object):
    # From fixed params
    JOINTS = 11
    POSE = 12
    POSITION = 13
    RPY = 14
    POSE_QUAT = 15
    LINEAR_POSE = 16

    # From actual pose
    SHIFT_POSE = 21
    SHIFT_LINEAR_POSE = 22

    # Trajectory
    EXECUTE_TRAJ = 31

    # Tools
    TOOL = 51

    # ADD-ONS
    DRAW_SPIRAL = 90


class ToolsCommand(object):
    DO_STUFF = 1


class MovingStyle(object):
    CLASSIC = 1
    LINEAR = 2


# - Exceptions

class RobotCommanderException(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message


class ArmCommanderException(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message


class ToolCommanderException(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message


class JogControllerException(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message
