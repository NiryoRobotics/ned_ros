#!/usr/bin/env python


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


class JogControllerException(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message
