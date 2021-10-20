#!/usr/bin/env python

# shutdown_manager.py
# Copyright (C) 2021 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.



import rospy
import threading

from niryo_robot_rpi.common.rpi_ros_utils import send_shutdown_command, send_reboot_command
from niryo_robot_rpi.common.abstract_shutdown_manager import AbstractShutdownManager

from std_msgs.msg import String


class ShutdownManager(AbstractShutdownManager):
    SHUTDOWN_TIMEOUT = 10

    def __init__(self):
        super(ShutdownManager, self).__init__()

        self.__turn_off_sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/turn_off_sound")

        self.__shutdown_event = threading.Event()
        self.__sound_status_sub = None
        self.__current_sound = ""

    def _shutdown(self):
        self.wait_end_of_sound()
        send_shutdown_command()

    def _reboot(self):
        self.wait_end_of_sound()
        send_reboot_command()

    def wait_end_of_sound(self):
        self.__shutdown_event.clear()
        self.__sound_status_sub = rospy.Subscriber("/niryo_robot_sound/sound", String, self.__callback_sound_state)
        self.__shutdown_event.wait(self.SHUTDOWN_TIMEOUT)

    def __callback_sound_state(self, msg):
        print msg.data
        if self.__current_sound == self.__turn_off_sound_name and msg.data == "":
            self.__shutdown_event.set()
        self.__current_sound = msg.data



