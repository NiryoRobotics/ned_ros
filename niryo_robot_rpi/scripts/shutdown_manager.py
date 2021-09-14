#!/usr/bin/env python

import rospy
import threading

from niryo_robot_rpi.rpi_ros_utils import send_reboot_command, send_shutdown_command

from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_msgs.srv import SetInt
from niryo_robot_msgs.srv import Trigger


class ShutdownManager(object):
    def __init__(self):
        rospy.logdebug("ShutdownManager - Entering in Init")

        rospy.Service('~shutdown_rpi', SetInt, self.callback_shutdown_rpi)

        self.__advertise_shutdown_service = rospy.ServiceProxy('/niryo_robot_status/advertise_shutdown', Trigger)

    def callback_shutdown_rpi(self, req):
        if req.value == 1:
            self.__advertise_shutdown_service.call()
            send_shutdown_command_thread = threading.Timer(1.0, send_shutdown_command)
            send_shutdown_command_thread.start()
            return CommandStatus.SUCCESS, 'Robot is shutting down'
        elif req.value == 2:
            self.__advertise_shutdown_service.call()
            send_reboot_command_thread = threading.Timer(1.0, send_reboot_command)
            send_reboot_command_thread.start()
            return CommandStatus.SUCCESS, 'Robot is rebooting'
        else:
            return CommandStatus.UNKNOWN_COMMAND, 'Incorrect value: 1 for shutdown, 2 for reboot'
