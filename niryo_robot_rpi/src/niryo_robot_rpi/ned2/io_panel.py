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
from collections import OrderedDict

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Libraries
from niryo_robot_rpi.common.abstract_io_panel import AbstractIOPanel
from niryo_robot_rpi.common.end_effector_panel import NiryoEndEffectorPanel

from .hardware.MAX11644 import MAX11644

from .mcp_io_objects import AnalogInput, AnalogOutput, McpIOManager


class IOPanel(AbstractIOPanel):
    def __init__(self, mcp_manager=None, adc=None, ):
        super(IOPanel, self).__init__()

        self.__mcp_manager = mcp_manager if mcp_manager is not None else McpIOManager()

        adc_param = rospy.get_param("~adc")
        self.__adc = adc if adc is not None else MAX11644(bus=adc_param["i2c_bus"], address=adc_param["address"],
                                                          v_ref_internal=adc_param["v_ref"])

        self.__end_effector_panel = NiryoEndEffectorPanel()
        self.__init_ios()

        self._publish_digital_io_state()

        self.__mcp_manager.register_on_change_callback(self._callback_digital_io_change, "io panel callback")

    def shutdown(self):
        super(IOPanel, self).shutdown()

    def __init_ios(self):
        self._digital_outputs = OrderedDict([(digital_output["name"], self.__mcp_manager.add_output(
            digital_output["pin"], digital_output["name"], reversed=digital_output["reverse"]))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"], self.__mcp_manager.add_input(digital_input["pin"],
                                                                                                 digital_input["name"]))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        self._digital_outputs[self.__end_effector_panel.digital_output.name] = self.__end_effector_panel.digital_output
        self._digital_inputs[self.__end_effector_panel.digital_input.name] = self.__end_effector_panel.digital_input

        self._digital_outputs = OrderedDict(sorted(self._digital_outputs.items()))
        self._digital_inputs = OrderedDict(sorted(self._digital_inputs.items()))

        self._analog_outputs = OrderedDict([(analog_output["name"],
                                             AnalogOutput(self.__mcp_manager.lock, analog_output["pin"],
                                                          analog_output["name"], analog_output["address"],
                                                          analog_output["i2c_bus"], analog_output["v_ref"],
                                                          analog_output["resolution"])) for analog_output in
                                            rospy.get_param("~analog_outputs")])

        self._analog_inputs = OrderedDict([(analog_input["name"],
                                            AnalogInput(self.__adc, self.__mcp_manager.lock, analog_input["pin"],
                                                        analog_input["name"], analog_input["dividing_bridge_factor"]))
                                           for analog_input in
                                           rospy.get_param("~analog_inputs")])

    def _callback_digital_io_change(self):
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
