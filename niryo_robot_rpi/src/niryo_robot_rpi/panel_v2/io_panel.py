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
from collections import OrderedDict, namedtuple

from std_msgs.msg import Bool

# Utils
from niryo_robot_utils import debounce

# Command Status
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_rpi.msg import I2CComponent

# Service
from niryo_robot_rpi.srv import ScanI2CBus, ScanI2CBusResponse

# Libraries
from ..common.abstract_io_panel import AbstractIOPanel
from ..common.end_effector_panel import NiryoEndEffectorPanel
from ..common.gpio_expander_adapters import GpioManager
from ..common.rpi_ros_utils import ping_i2c

from .hardware.MAX11644 import MAX11644
from .mcp_io_objects import AnalogInput, AnalogOutput, McpIOManager

I2CObject = namedtuple("I2CObject", ['info', 'bus', 'address'])


class IOPanel(AbstractIOPanel):

    def __init__(
        self,
        mcp_manager=None,
        adc=None,
    ):
        super(IOPanel, self).__init__()

        self.__mcp_manager = mcp_manager if mcp_manager is not None else McpIOManager()

        adc_param = rospy.get_param("~adc")
        self.__adc = adc if adc is not None else MAX11644(
            bus=adc_param["i2c_bus"], address=adc_param["address"], v_ref_internal=adc_param["v_ref"])
        self.__end_effector_panel = NiryoEndEffectorPanel(self._callback_digital_io_change)
        self.__init_ios()

        self._publish_digital_io_state()

        self.__mcp_manager.register_on_change_callback(self._callback_digital_io_change, "io panel callback")

        self.__sound_card_i2C = I2CObject(info='Sound Card',
                                          address=rospy.get_param('~sound_card/i2c_address'),
                                          bus=rospy.get_param('~sound_card/i2c_bus'))
        rospy.Service("~scan_i2c_bus", ScanI2CBus, self.__callback_scan_i2c_bus)

        # This pin is only available for the ned3pro
        if rospy.has_param('~monitor_12v_pin'):
            self.__gpio_manager = GpioManager()
            self.__estop_status_pub = rospy.Publisher('/niryo_robot_rpi/estop_status', Bool, queue_size=10)
            self.__monitor_12v = self.__gpio_manager.add_input(rospy.get_param('~monitor_12v_pin'), 'monitor 12v')
            self.__monitor_12v.on_change(self.__on_estop_change_callback)

    def shutdown(self):
        super(IOPanel, self).shutdown()

    def __init_ios(self):
        self._digital_outputs = OrderedDict([(digital_output["name"],
                                              self.__mcp_manager.add_output(digital_output["pin"],
                                                                            digital_output["name"],
                                                                            reversed=digital_output["reverse"]))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"],
                                             self.__mcp_manager.add_input(digital_input["pin"],
                                                                          digital_input["name"],
                                                                          digital_input["reverse"]))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        self._digital_outputs[self.__end_effector_panel.digital_output.name] = self.__end_effector_panel.digital_output
        self._digital_inputs[self.__end_effector_panel.digital_input.name] = self.__end_effector_panel.digital_input

        self._digital_outputs = OrderedDict(sorted(self._digital_outputs.items()))
        self._digital_inputs = OrderedDict(sorted(self._digital_inputs.items()))

        self._analog_outputs = OrderedDict([(analog_output["name"],
                                             AnalogOutput(self.__mcp_manager.lock,
                                                          analog_output["pin"],
                                                          analog_output["name"],
                                                          analog_output["address"],
                                                          analog_output["i2c_bus"],
                                                          analog_output["v_ref"],
                                                          analog_output["resolution"]))
                                            for analog_output in rospy.get_param("~analog_outputs")])

        self._analog_inputs = OrderedDict([(analog_input["name"],
                                            AnalogInput(self.__adc,
                                                        self.__mcp_manager.lock,
                                                        analog_input["pin"],
                                                        analog_input["name"],
                                                        analog_input["dividing_bridge_factor"]))
                                           for analog_input in rospy.get_param("~analog_inputs")])

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
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR, "No IO found for {}".format(req.name))
        self._publish_analog_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))

    def __callback_scan_i2c_bus(self, _req):
        resp = ScanI2CBusResponse()

        for i2c_object in [self.__mcp_manager.mcp, self.__adc, self.__sound_card_i2C] + list(
                self._analog_outputs.values()):
            i2C_component = I2CComponent(info=str(i2c_object), bus=int(i2c_object.bus), address=int(i2c_object.address))
            if ping_i2c(i2c_object.bus, i2c_object.address):
                resp.detected.append(i2C_component)
            else:
                resp.missing.append(i2C_component)
                rospy.logwarn('Missing ic2 component: bus {}, address {}'.format(int(i2c_object.bus),
                                                                                 int(i2c_object.address)))
        resp.is_ok = not resp.missing

        if resp.is_ok:
            resp.status = CommandStatus.SUCCESS
            resp.message = "I2C scan succeed"
        else:
            resp.is_ok = CommandStatus.MISSING_I2C
            resp.message = "I2C scan failed"

        return resp

    @debounce(2.0, lambda self, value: value != 1)
    def __on_estop_change_callback(self, value):
        msg = Bool(value == 1)
        self.__estop_status_pub.publish(msg)
