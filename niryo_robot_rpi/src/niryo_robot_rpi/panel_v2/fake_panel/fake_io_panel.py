# fake_io_panel.py
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

from ...common.io_objects import PinMode
from ...common.abstract_io_panel import AbstractIOPanel
from ...common.end_effector_panel import NiryoEndEffectorPanel

from .fake_io_objects import FakeDigitalIO, FakeAnalogIO


class FakeIOPanel(AbstractIOPanel):

    def __init__(self):
        super(FakeIOPanel, self).__init__()

        self.__end_effector_panel = NiryoEndEffectorPanel(self._publish_digital_io_state)

        self._digital_outputs = OrderedDict([(digital_output["name"],
                                              FakeDigitalIO(digital_output["name"], PinMode.DIGITAL_OUTPUT))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"],
                                             FakeDigitalIO(digital_input["name"], PinMode.DIGITAL_INPUT))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        self._digital_outputs[self.__end_effector_panel.digital_output.name] = self.__end_effector_panel.digital_output
        self._digital_inputs[self.__end_effector_panel.digital_input.name] = self.__end_effector_panel.digital_input

        self._digital_outputs = OrderedDict(sorted(self._digital_outputs.items()))
        self._digital_inputs = OrderedDict(sorted(self._digital_inputs.items()))

        self._analog_outputs = OrderedDict([(analog_output["name"],
                                             FakeAnalogIO(analog_output["name"], PinMode.ANALOG_OUTPUT))
                                            for analog_output in rospy.get_param("~analog_outputs")])

        self._analog_inputs = OrderedDict([(analog_input["name"],
                                            FakeAnalogIO(analog_input["name"], PinMode.ANALOG_INPUT))
                                           for analog_input in rospy.get_param("~analog_inputs")])

        self._publish_digital_io_state()

    def _callback_set_digital_io(self, req):
        if req.name in self._digital_outputs:
            self._digital_outputs[req.name].value = req.value
        elif req.name in self._digital_inputs:
            self._digital_inputs[req.name].value = req.value
        elif req.name in self._analog_inputs or req.name in self._analog_outputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "{} is not a digital io.".format(req.name))
        else:
            # No pin found
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR, "No IO found for {}".format(req.name))
        self._publish_digital_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))
