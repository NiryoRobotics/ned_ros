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

from .fake_io_objects import FakeDigitalIO

from ...common.abstract_io_panel import AbstractDigitalIOPanel
from ...common.io_objects import PinMode


class FakeIOPanel(AbstractDigitalIOPanel):
    def __init__(self):
        super(FakeIOPanel, self).__init__()
        self._init_ios()
        self._publish_digital_io_state()

    def _init_ios(self):
        self._digital_outputs = OrderedDict([(digital_output["name"], FakeDigitalIO(digital_output["name"],
                                                                                    PinMode.DIGITAL_OUTPUT))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"], FakeDigitalIO(digital_input["name"],
                                                                                  PinMode.DIGITAL_INPUT))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        for switch in rospy.get_param("~switches"):
            self._switches_name.append(switch["name"])
            self._digital_outputs[switch["name"]] = FakeDigitalIO(switch["name"], PinMode.DIGITAL_OUTPUT)

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
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No IO found for {}".format(req.name))
        self._publish_digital_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))
