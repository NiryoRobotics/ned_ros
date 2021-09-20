#!/usr/bin/env python

# digital_io_panel.py
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

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_rpi.msg import DigitalIO as DigitalIOMsg
from niryo_robot_rpi.msg import DigitalIOState, AnalogIO, AnalogIOState
from niryo_robot_rpi.srv import GetDigitalIO, GetAnalogIO
from niryo_robot_rpi.srv import SetDigitalIO, SetAnalogIO
from niryo_robot_rpi.srv import SetIOMode

from niryo_robot_rpi.fake_io_objects import FakeDigitalIO, FakeAnalogIO
from niryo_robot_rpi.io_objects import PinMode


class IOPanel(object):
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
                                         "No GPIO found with this pin number (" + str(req.pin) + ")")

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


class McpIOPanel(IOPanel):
    def __init__(self, mcp=None, adc=None, lock=None):
        import RPi.GPIO as GPIO
        from Adafruit_ADS1x15 import ADS1115
        from niryo_robot_rpi.MCP23017 import MCP23017
        from niryo_robot_rpi.Adafruit_I2C import Adafruit_I2C
        from wifi_button import WifiButton

        super(McpIOPanel, self).__init__()
        self.__mcp = mcp if mcp is not None else MCP23017(address=rospy.get_param("~mcp/address"),
                                                          busnum=rospy.get_param("~mcp/i2c_bus"))

        self.__adc = adc if adc is not None else ADS1115(i2c=Adafruit_I2C, busnum=rospy.get_param("~ads/i2c_bus"))

        if lock is None:
            lock = Lock()
        self.__lock = lock

        self.__init_ios(lock)
        self.__wifi_button = WifiButton(mcp=self.__mcp, lock=lock)

        self._publish_digital_io_state()
        self.check_interrupt_flag_security_frequency = rospy.get_param("~check_interrupt_flag_security_frequency")
        rospy.Timer(rospy.Duration(1.0 / self.check_interrupt_flag_security_frequency),
                    self._check_interrupt_flag_security)

        interrupt_BCM = rospy.get_param("~mcp/interrupt_bcm")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(interrupt_BCM, GPIO.IN)
        GPIO.add_event_detect(interrupt_BCM, GPIO.FALLING, callback=self.interrupt_callback)

        self.read_digital_inputs()

    def __del__(self):
        GPIO.cleanup()

    def __init_ios(self, lock):
        from niryo_robot_rpi.mcp_io_objects import DigitalInput, DigitalOutput, AnalogInput, AnalogOutput

        self._digital_outputs = OrderedDict([(digital_output["name"], DigitalOutput(self.__mcp, lock,
                                                                                    digital_output["pin"],
                                                                                    digital_output["name"],
                                                                                    reversed=digital_output["reverse"]))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"], DigitalInput(self.__mcp,
                                                                                 lock, digital_input["pin"],
                                                                                 digital_input["name"]))
                                            for digital_input in rospy.get_param("~digital_inputs")])

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
        from niryo_robot_rpi.MCP23017 import RegistersMCP23017

        with self.__lock:
            pending_interrupts = self.__mcp.readU8(RegistersMCP23017.INTFA)

        if pending_interrupts:
            self.read_digital_inputs()


class DigitalIOPanel(IOPanel):

    def __init__(self, lock=None):
        super(DigitalIOPanel, self).__init__()

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


class DigitalRpiIOPanel(DigitalIOPanel):
    def __init__(self, lock=None):
        import RPi.GPIO as GPIO

        super(DigitalRpiIOPanel, self).__init__()

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

    def _init_ios(self):
        from niryo_robot_rpi.rpi_io_objects import DigitalPin

        self._digital_outputs = OrderedDict([(digital_output["name"], DigitalPin(self.__lock,
                                                                                 digital_output["pin"],
                                                                                 digital_output["name"],
                                                                                 PinMode.DIGITAL_OUTPUT))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"], DigitalPin(self.__lock,
                                                                               digital_input["pin"],
                                                                               digital_input["name"],
                                                                               PinMode.DIGITAL_INPUT))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        for switch in rospy.get_param("~switches"):
            self._switches_name.append(switch["name"])
            self._digital_outputs[switch["name"]] = DigitalPin(self.__lock, switch["pin"], switch["name"],
                                                               PinMode.DIGITAL_OUTPUT)


class FakeIOPanel(IOPanel):
    def __init__(self):
        super(FakeIOPanel, self).__init__()

        self._digital_outputs = OrderedDict(
            [(digital_output["name"], FakeDigitalIO(digital_output["name"], PinMode.DIGITAL_OUTPUT))
             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict(
            [(digital_input["name"], FakeDigitalIO(digital_input["name"], PinMode.DIGITAL_INPUT))
             for digital_input in rospy.get_param("~digital_inputs")])

        self._analog_outputs = OrderedDict(
            [(analog_output["name"], FakeAnalogIO(analog_output["name"], PinMode.ANALOG_OUTPUT))
             for analog_output in rospy.get_param("~analog_outputs")])

        self._analog_inputs = OrderedDict(
            [(analog_input["name"], FakeAnalogIO(analog_input["name"], PinMode.ANALOG_INPUT))
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
            return self._create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                         "No IO found for {}".format(req.name))
        self._publish_digital_io_state()
        return self._create_response(CommandStatus.SUCCESS, "Successfully set value for {}".format(req.name))


class FakeDigitalIOPanel(DigitalIOPanel):
    def __init__(self):
        super(FakeDigitalIOPanel, self).__init__()
        self._init_ios()
        self._publish_digital_io_state()

    def _init_ios(self):
        from niryo_robot_rpi.fake_io_objects import FakeRpiDigitalIO
        self._digital_outputs = OrderedDict([(digital_output["name"], FakeRpiDigitalIO(digital_output["name"],
                                                                                       PinMode.DIGITAL_OUTPUT))
                                             for digital_output in rospy.get_param("~digital_outputs")])

        self._digital_inputs = OrderedDict([(digital_input["name"], FakeRpiDigitalIO(digital_input["name"],
                                                                                     PinMode.DIGITAL_INPUT))
                                            for digital_input in rospy.get_param("~digital_inputs")])

        for switch in rospy.get_param("~switches"):
            self._switches_name.append(switch["name"])
            self._digital_outputs[switch["name"]] = FakeRpiDigitalIO(switch["name"], PinMode.DIGITAL_OUTPUT)

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


if __name__ == '__main__':
    rospy.init_node('io_panel')
    # IOPanel()
    rospy.spin()
