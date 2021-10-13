#!/usr/bin/env python

import rospy
import threading

from niryo_robot_rpi.rpi_ros_utils import send_reboot_command, send_shutdown_command

from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.srv import SetInt
from niryo_robot_msgs.srv import Trigger
from std_msgs.msg import String


class ShutdownManager(object):
    def __init__(self):
        rospy.logdebug("ShutdownManager - Entering in Init")

        rospy.Service('~shutdown_rpi', SetInt, self.callback_shutdown_rpi)

        self.__advertise_shutdown_service = rospy.ServiceProxy('/niryo_robot_status/advertise_shutdown', Trigger)

    def callback_shutdown_rpi(self, req):
        if req.value == 1:
            self.__advertise_shutdown_service.call()
            send_shutdown_command_thread = threading.Timer(1.0, self._shutdown)
            send_shutdown_command_thread.start()
            return CommandStatus.SUCCESS, 'Robot is shutting down'
        elif req.value == 2:
            self.__advertise_shutdown_service.call()
            send_reboot_command_thread = threading.Timer(1.0, self._shutdown)
            send_reboot_command_thread.start()
            return CommandStatus.SUCCESS, 'Robot is rebooting'
        else:
            return CommandStatus.UNKNOWN_COMMAND, 'Incorrect value: 1 for shutdown, 2 for reboot'

    @staticmethod
    def _shutdown():
        print('shutdown')
        send_shutdown_command()


class ShutdownManagerNed2(ShutdownManager):
    SHUTDOWN_TIMEOUT = 10

    def __init__(self):
        super(ShutdownManagerNed2, self).__init__()

        self.__turn_off_sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/turn_off_sound")

        self.__shutdown_event = threading.Event()
        self.__sound_status_sub = None
        self.__current_sound = ""

    def _shutdown(self):
        self.wait_end_of_sound()
        send_shutdown_command()

    def wait_end_of_sound(self):
        self.__shutdown_event.clear()
        self.__sound_status_sub = rospy.Subscriber("/niryo_robot_sound/sound", String, self.__callback_sound_state)
        self.__shutdown_event.wait(self.SHUTDOWN_TIMEOUT)

    def __callback_sound_state(self, msg):
        print msg.data
        if self.__current_sound == self.__turn_off_sound_name and msg.data == "":
            self.__shutdown_event.set()
        self.__current_sound = msg.data



