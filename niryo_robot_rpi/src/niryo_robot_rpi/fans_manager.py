#!/usr/bin/env python

# fans_manager.py
# Copyright (C) 2017 Niryo
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

from niryo_robot_msgs.msg import HardwareStatus

import RPi.GPIO as GPIO


class FansManager:
    def __init__(self):
        self._fans_list = rospy.get_param("/niryo_robot_rpi/fans")
        self.setup_fans()

        self.hardware_status_subscriber = rospy.Subscriber(
            '/niryo_robot_hardware_interface/hardware_status', HardwareStatus, self.callback_hardware_status)

    def setup_fans(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for fan in self._fans_list:
            GPIO.setup(fan["gpio"], GPIO.OUT)

        for fan in self._fans_list:
            self.set_fans(False, fan["gpio"])
        rospy.loginfo("Fan Manager - Started")

    @staticmethod
    def set_fans(activate, gpio):
        try:
            GPIO.output(gpio, GPIO.HIGH if activate else GPIO.LOW)
        except RuntimeError:
            pass

    def callback_hardware_status(self, msg):
        for fan in self._fans_list:
            if msg.rpi_temperature >= fan["temperature_on_threshold"]:
                self.set_fans(True, fan["gpio"])
            elif msg.rpi_temperature <= fan["temperature_off_threshold"]:
                self.set_fans(False, fan["gpio"])
