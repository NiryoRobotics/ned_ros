#!/usr/bin/env python


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
