# abstract_io_panel.py
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
from threading import Lock

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_rpi.msg import DigitalIO as DigitalIOMsg
from niryo_robot_rpi.msg import DigitalIOState, AnalogIO, AnalogIOState
from niryo_robot_rpi.srv import GetDigitalIO, GetAnalogIO
from niryo_robot_rpi.srv import SetDigitalIO, SetAnalogIO
from niryo_robot_rpi.srv import SetIOMode

from .io_objects import PinMode


class AbstractIOPanel(object):
    def __init__(self):
        self._digital_outputs = OrderedDict()
        self._digital_inputs = OrderedDict()
        self._analog_outputs = OrderedDict()
        self._analog_inputs = OrderedDict()

        self.digital_io_publisher = rospy.Publisher('/niryo_robot_rpi/digital_io_state',
                                                    DigitalIOState, queue_size=10, latch=True)

        self._publish_io_state_frequency = rospy.get_param("~publish_io_state_frequency")
        self.analog_io_publisher = rospy.Publisher('/niryo_robot_rpi/analog_io_state',
                                                   AnalogIOState, queue_size=10, latch=True)
        self._publish_analog_io_state_timer = rospy.Timer(rospy.Duration(1.0 / self._publish_io_state_frequency),
                                                          self._publish_analog_io_state_cb)

        self.set_io_state_server = rospy.Service('~get_analog_io', GetAnalogIO, self._callback_get_analog_io)
        self.set_io_state_server = rospy.Service('~get_digital_io', GetDigitalIO, self._callback_get_digital_io)

        self.set_io_state_server = rospy.Service('~set_analog_io', SetAnalogIO, self._callback_set_analog_io)
        self.set_io_state_server = rospy.Service('~set_digital_io', SetDigitalIO, self._callback_set_digital_io)

        self.set_io_mode_server = rospy.Service('~set_digital_io_mode', SetIOMode, self._callback_set_io_mode)

    def _publish_digital_io_state(self):
        msg = DigitalIOState()
        try:
            for digital_input in self._digital_inputs.values():
                msg.digital_inputs.append(
                    DigitalIOMsg(digital_input.name, digital_input.value))

            for digital_output in self._digital_outputs.values():
                msg.digital_outputs.append(DigitalIOMsg(digital_output.name, digital_output.value))
        except RuntimeError:
            return

        try:
            self.digital_io_publisher.publish(msg)
        except rospy.ROSException as e:
            rospy.logerr("IO Panel - {}".format(e))
            return

    def _publish_analog_io_state_cb(self, _event=None):
        self._publish_analog_io_state()

    def _publish_analog_io_state(self):
        msg = AnalogIOState()
        try:
            for analog_input in self._analog_inputs.values():
                msg.analog_inputs.append(AnalogIO(analog_input.name, analog_input.value))

            for analog_output in self._analog_outputs.values():
                msg.analog_outputs.append(AnalogIO(analog_output.name, analog_output.value))
        except RuntimeError:
            return

        try:
            self.analog_io_publisher.publish(msg)
        except rospy.ROSException as e:
            rospy.logerr("IO Panel - {}".format(e))
            return

    @staticmethod
    def _create_response(status, message):
        return {'status': status, 'message': message}

    def _callback_get_analog_io(self, req):
        if req.name in self._analog_inputs:
            value = self._analog_inputs[req.name].value
        elif req.name in self._analog_outputs:
            value = self._analog_outputs[req.name].value
        elif req.name in self._digital_outputs or req.name in self._digital_inputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "{} is not an analog IO, but a digital IO".format(req.name))
        else:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No GPIO found with this pin name (" + str(req.name) + ")")

        return {'status': CommandStatus.SUCCESS, 'message': 'OK', 'value': value}

    def _callback_get_digital_io(self, req):
        if req.name in self._digital_inputs:
            value = self._digital_inputs[req.name].value
        elif req.name in self._digital_outputs:
            value = self._digital_outputs[req.name].value
        elif req.name in self._analog_outputs or req.name in self._analog_inputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "{} is not a digital IO, but a analog IO".format(req.name))
        else:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No GPIO found with this pin number (" + str(req.name) + ")")

        return {'status': CommandStatus.SUCCESS, 'message': 'OK', 'value': value}

    def _callback_set_analog_io(self, req):
        if req.name in self._analog_outputs:
            self._analog_outputs[req.name].value = req.value
        elif req.name in self._analog_inputs:
            self._analog_inputs[req.name].value = req.value

        elif req.name in self._digital_inputs or req.name in self._digital_outputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "{} is not an analog IO, but a digital IO".format(req.name))
        else:
            # No pin found
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No IO found for {}".format(req.name))
        self._publish_analog_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))

    def _callback_set_digital_io(self, req):
        if req.name in self._digital_outputs:
            self._digital_outputs[req.name].value = req.value
        elif req.name in self._analog_inputs or req.name in self._analog_outputs or req.name in self._digital_inputs:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "{} is not a digital output.".format(req.name))
        else:
            # No pin found
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No IO found for {}".format(req.name))
        self._publish_digital_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))

    def _callback_set_io_mode(self, _req):
        return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                     "You cannot change the IO mode on this robot version")

    def shutdown(self):
        for do in self._digital_outputs.values():
            do.value = False

        for ao in self._analog_outputs.values():
            ao.value = 0


class AbstractDigitalIOPanel(AbstractIOPanel):

    def __init__(self, lock=None):
        super(AbstractDigitalIOPanel, self).__init__()

        if lock is None:
            lock = Lock()
        self.__lock = lock

        self._switches_name = []
        self._publish_analog_io_state_timer.shutdown()
        self._publish_analog_io_state()

        self._publish_digital_io_state_timer = rospy.Timer(rospy.Duration(1.0 / self._publish_io_state_frequency),
                                                           self._publish_digital_io_state_cb)

    def _publish_digital_io_state_cb(self, _event=None):
        self._publish_digital_io_state()

    def _callback_set_io_mode(self, req):
        if req.name in self._switches_name:
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "Can't change mode for switch pin, mode is fixed to OUTPUT")
        elif req.name in self._digital_outputs:
            if req.mode == req.INPUT:
                new_input = self._digital_outputs.pop(req.name)
                new_input.mode = PinMode.DIGITAL_INPUT
                self._digital_inputs[req.name] = new_input
                self._digital_inputs = OrderedDict(sorted(self._digital_inputs.items()))
            else:
                return self._create_response(CommandStatus.SUCCESS,
                                             "IO already set as OUTPUT")
        elif req.name in self._digital_inputs:
            if req.mode == req.OUTPUT:
                new_output = self._digital_inputs.pop(req.name)
                new_output.mode = PinMode.DIGITAL_OUTPUT
                self._digital_outputs[req.name] = new_output
                self._digital_outputs = OrderedDict(sorted(self._digital_outputs.items()))
            else:
                return self._create_response(CommandStatus.SUCCESS,
                                             "IO already set as INPUT")
        else:
            # No pin found
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No IO found for {}".format(req.name))

        self._publish_digital_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))
