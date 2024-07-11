# io_panel.py
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
from threading import Lock
from collections import OrderedDict
import RPi.GPIO as GPIO

from ..common.abstract_io_panel import AbstractDigitalIOPanel
from ..common.io_objects import PinMode
from .rpi_io_objects import DigitalPin


class IOPanel(AbstractDigitalIOPanel):

    def __init__(self, lock=None):
        super(IOPanel, self).__init__()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        if lock is None:
            lock = Lock()
        self.__lock = lock

        self._switches_name = []
        self._init_ios()
        self._publish_digital_io_state()

    def __del__(self):
        GPIO.cleanup()

    def shutdown(self):
        super(IOPanel, self).shutdown()
        GPIO.cleanup()

    def _init_ios(self):
        self._digital_outputs = OrderedDict([(digital_output["name"],
                                              DigitalPin(self.__lock,
                                                         digital_output["pin"],
                                                         digital_output["name"],
                                                         PinMode.DIGITAL_OUTPUT))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"],
                                             DigitalPin(self.__lock,
                                                        digital_input["pin"],
                                                        digital_input["name"],
                                                        PinMode.DIGITAL_INPUT))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        for switch in rospy.get_param("~switches"):
            self._switches_name.append(switch["name"])
            self._digital_outputs[switch["name"]] = DigitalPin(self.__lock,
                                                               switch["pin"],
                                                               switch["name"],
                                                               PinMode.DIGITAL_OUTPUT)
