#!/usr/bin/env python

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

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Libraries
from niryo_robot_rpi.common.abstract_io_panel import AbstractIOPanel
from niryo_robot_rpi.common.end_effector_panel import NiryoEndEffectorPanel

from .hardware.MCP23017 import RegistersMCP23017
from .hardware.MCP23017 import MCP23017
from .hardware.Adafruit_I2C import Adafruit_I2C

from Adafruit_ADS1x15 import ADS1115

from .mcp_io_objects import DigitalInput, DigitalOutput, AnalogInput, AnalogOutput
from .wifi_button import WifiButton


class IOPanel(AbstractIOPanel):
    def __init__(self, mcp=None, adc=None, lock=None):
        super(IOPanel, self).__init__()
        self.__mcp = mcp if mcp is not None else MCP23017(address=rospy.get_param("~mcp/address"),
                                                          busnum=rospy.get_param("~mcp/i2c_bus"))

        self.__adc = adc if adc is not None else ADS1115(i2c=Adafruit_I2C, busnum=rospy.get_param("~ads/i2c_bus"))

        if lock is None:
            lock = Lock()
        self.__lock = lock

        self.__wifi_button = WifiButton(mcp=self.__mcp, lock=lock)
        self.__end_effector_panel = NiryoEndEffectorPanel()
        self.__init_ios(lock)

        self._publish_digital_io_state()
        self.check_interrupt_flag_security_frequency = rospy.get_param("~check_interrupt_flag_security_frequency")
        rospy.Timer(rospy.Duration(1.0 / self.check_interrupt_flag_security_frequency),
                    self._check_interrupt_flag_security)

        interrupt_BCM = rospy.get_param("~mcp/interrupt_bcm")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(interrupt_BCM, GPIO.IN)
        GPIO.add_event_detect(interrupt_BCM, GPIO.RISING, callback=self.interrupt_callback)

        self.read_digital_inputs()

    def __del__(self):
        GPIO.cleanup()

    def shutdown(self):
        super(IOPanel, self).shutdown()
        self.__wifi_button.shutdown()
        GPIO.cleanup()

    def __init_ios(self, lock):
        self._digital_outputs = OrderedDict([(digital_output["name"], DigitalOutput(self.__mcp, lock,
                                                                                    digital_output["pin"],
                                                                                    digital_output["name"],
                                                                                    reversed=digital_output["reverse"]))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"], DigitalInput(self.__mcp,
                                                                                 lock, digital_input["pin"],
                                                                                 digital_input["name"]))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        self._digital_outputs[self.__end_effector_panel.digital_output.name] = self.__end_effector_panel.digital_output
        self._digital_inputs[self.__end_effector_panel.digital_input.name] = self.__end_effector_panel.digital_input

        self._digital_outputs = OrderedDict(sorted(self._digital_outputs.items()))
        self._digital_inputs = OrderedDict(sorted(self._digital_inputs.items()))

        self._analog_outputs = OrderedDict([(analog_output["name"],
                                             AnalogOutput(lock, analog_output["pin"], analog_output["name"],
                                                          analog_output["address"], analog_output["i2c_bus"],
                                                          analog_output["v_ref"], analog_output["resolution"]))
                                            for analog_output in rospy.get_param("~analog_outputs")])

        reading_factor = rospy.get_param("~ads/reading_factor")
        self._analog_inputs = OrderedDict([(analog_input["name"], AnalogInput(self.__adc, lock, analog_input["pin"],
                                                                              analog_input["name"], reading_factor))
                                           for analog_input in rospy.get_param("~analog_inputs")])

    def interrupt_callback(self, _channel=None):
        # self.__mcp.debug()
        self.read_digital_inputs()

    def read_digital_inputs(self):
        with self.__lock:
            gpio_values = self.__mcp.read_all_gpios()
            for digital_input in self._digital_inputs.values():
                digital_input.value = bool((gpio_values >> digital_input.pin) & 0b1)
            self.__wifi_button.set((gpio_values >> self.__wifi_button.pin) & 0b1)

            self._publish_digital_io_state()

    def _callback_set_analog_io(self, req):
        if req.name in self._analog_outputs:
            self._analog_outputs[req.name].value = req.value
        elif req.name in self._analog_inputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "You cannot set a state of an analog input outside the simulation mode")
        elif req.name in self._digital_inputs or req.name in self._digital_outputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "{} is not an analog IO, but a digital IO".format(req.name))
        else:
            # No pin found
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No IO found for {}".format(req.name))
        self._publish_analog_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))

    def _check_interrupt_flag_security(self, _event):
        with self.__lock:
            pending_interrupts = self.__mcp.readU8(RegistersMCP23017.INTFA)

        if pending_interrupts:
            self.read_digital_inputs()
