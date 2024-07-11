# mcp_io_objects.py
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

from threading import Thread, Lock
import RPi.GPIO as GPIO
import rospy

from ..common.io_objects import PinMode, NiryoIO

from .hardware.DACx0501 import DACx0501
from .hardware.MCP23017 import MCP23017, RegistersMCP23017


class McpIOManager(object):

    def __init__(self, mcp=None):
        self.__mcp = mcp if mcp is not None else MCP23017(address=rospy.get_param("~mcp/address"),
                                                          busnum=rospy.get_param("~mcp/i2c_bus"))

        self.__lock = Lock()

        self.__inputs = {}
        self.__outputs = {}

        self.__callbacks = {}

        interrupt_BCM = rospy.get_param("~mcp/interrupt_bcm")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(interrupt_BCM, GPIO.IN)
        GPIO.add_event_detect(interrupt_BCM, GPIO.FALLING, callback=self.interrupt_callback)

        self.check_interrupt_flag_security_frequency = rospy.get_param("~check_interrupt_flag_security_frequency")
        rospy.Timer(rospy.Duration(1.0 / self.check_interrupt_flag_security_frequency),
                    self._check_interrupt_flag_security)

        self.read_digital_inputs()

    def __del__(self):
        GPIO.cleanup()

    @property
    def mcp(self):
        return self.__mcp

    @property
    def lock(self):
        return self.__lock

    @staticmethod
    def shutdown():
        GPIO.cleanup()

    def add_output(self, pin, name, reversed=False):
        self.__outputs[name] = DigitalOutput(self.__mcp, self.__lock, pin, name, reversed)
        return self.__outputs[name]

    def add_input(self, pin, name, reverse_polarity=True):
        self.__inputs[name] = DigitalInput(self.__mcp, self.__lock, pin, name, reverse_polarity)
        self.read_digital_inputs()
        return self.__inputs[name]

    def add_button(self, pin, name, pullup=True, reverse_polarity=False):
        self.__inputs[name] = Button(self.__mcp, self.__lock, pin, name, pullup, reverse_polarity)
        self.read_digital_inputs()
        return self.__inputs[name]

    def add_led(self, pin, name):
        self.__outputs[name] = Led(self.__mcp, self.__lock, pin, name)
        return self.__outputs[name]

    def add_fan(self, pin, name, temperature_on_threshold, temperature_off_threshold):
        self.__outputs[name] = Fan(self.__mcp,
                                   self.__lock,
                                   pin,
                                   name,
                                   temperature_on_threshold,
                                   temperature_off_threshold)
        return self.__outputs[name]

    def read_digital_inputs(self):
        with self.__lock:
            gpio_values = self.__mcp.read_all_gpios()
            for digital_input in self.__inputs.values():
                digital_input.value = bool((gpio_values >> digital_input.pin) & 0b1)
        self.advertise_callbacks()

    def interrupt_callback(self, _channel=None):
        # self.__mcp.debug()
        self.read_digital_inputs()

    def register_on_change_callback(self, cb, cb_id):
        self.__callbacks[cb_id] = cb

    def unregister_on_change_callback(self, cb_id):
        self.__callbacks[cb_id].pop()

    def advertise_callbacks(self):
        for cb in self.__callbacks.values():
            cb()

    def _check_interrupt_flag_security(self, _event):
        with self.__lock:
            pending_interrupts = self.__mcp.readU8(RegistersMCP23017.INTFA)
        if pending_interrupts:
            self.read_digital_inputs()


class DigitalOutput(NiryoIO):

    def __init__(self, mcp, lock, pin, name, reversed=False):
        super(DigitalOutput, self).__init__(lock, pin, name)

        self._mcp = mcp
        self._mode = PinMode.DIGITAL_OUTPUT
        self._reversed = reversed

        with self._lock:
            self._mcp.enable_interrupt(self._pin, False)
            self._mcp.config(self._pin, self._mcp.OUTPUT)

        self.value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        with self._lock:
            limited_value = max(0, min(int(value), 1))
            self._mcp.output(self._pin, int(not limited_value) if self._reversed else limited_value)
            self._value = limited_value


class Fan(DigitalOutput):

    def __init__(self, mcp, lock, pin, name, temperature_on_threshold, temperature_off_threshold):
        super(Fan, self).__init__(mcp, lock, pin, name)

        self.temperature_on_threshold = temperature_on_threshold
        self.temperature_off_threshold = temperature_off_threshold

    def update(self, temperature):
        if temperature >= self.temperature_on_threshold:
            if not self._value:
                self.value = 1
        elif temperature <= self.temperature_off_threshold:
            if self._value:
                self.value = 0


class Led(DigitalOutput):

    def __init__(self, mcp, lock, pin, name):
        super(Led, self).__init__(mcp, lock, pin, name)

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def reverse_state(self):
        self.value = 0 if self.value else 1


class DigitalInput(NiryoIO):

    def __init__(self, mcp, lock, pin, name, reverse_polarity=True):
        super(DigitalInput, self).__init__(lock, pin, name)

        self._mcp = mcp
        self._mode = PinMode.DIGITAL_INPUT

        with self._lock:
            self._mcp.config(self._pin, self._mcp.INPUT)
            self._mcp.enable_interrupt(self._pin, True)
            self._mcp.reverse_input_polarity(self._pin, reverse_polarity)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value

    def read_value(self):
        with self._lock:
            self._value = self._mcp.input(self._pin)
            return self._value


class Button(DigitalInput):
    PRESSED_VALUE = 1
    RELEASED_VALUE = 0

    def __init__(self, mcp, lock, pin, name, pullup=True, reverse_polarity=False):
        self.__on_press_function = None
        self.__on_release_function = None

        super(Button, self).__init__(mcp, lock, pin, name, reverse_polarity)
        with self._lock:
            self._mcp.pullup(self._pin, pullup)

        self._value = self.RELEASED_VALUE

    def __del__(self):
        self.__on_press_function = None
        self.__on_release_function = None

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        if value != self._value:
            self._value = value
            if self.__on_release_function and not self._value:
                Thread(target=self.__on_release_function).start()
            elif self.__on_press_function and self._value:
                Thread(target=self.__on_press_function).start()

    def on_press(self, function):
        self.__on_press_function = function

    def disable_on_press(self):
        self.__on_press_function = None

    def on_release(self, function):
        self.__on_release_function = function

    def disable_on_release(self):
        self.__on_release_function = None


class AnalogInput(NiryoIO):

    def __init__(self, adc, lock, pin, name, dividing_bridge_factor=1):
        super(AnalogInput, self).__init__(lock, pin, name)

        self.__adc = adc
        self.__mode = PinMode.ANALOG_INPUT
        self.__dividing_bridge_factor = dividing_bridge_factor

    @property
    def value(self):
        with self._lock:
            self._value = round(self.__adc.get_voltage(self._pin, self.__dividing_bridge_factor), 2)
            return self._value


class AnalogOutput(NiryoIO):

    def __init__(self, lock, pin, name, address, bus_i2c, v_ref, resolution):
        super(AnalogOutput, self).__init__(lock, pin, name)

        self.__i2c_bus = bus_i2c
        self.__i2c_address = address

        self.__dac = DACx0501(bus=bus_i2c, address=address, v_ref=v_ref, resolution=resolution)
        self.__mode = PinMode.ANALOG_OUTPUT

    def __str__(self):
        return 'DACx0501 {}, bus {}, address {}, value {}'.format(self._name,
                                                                  self.__i2c_bus,
                                                                  self.__i2c_address,
                                                                  self._value)

    @property
    def bus(self):
        return self.__i2c_bus

    @property
    def address(self):
        return self.__i2c_address

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (int, float))
        with self._lock:
            limited_voltage = max(0, min(value, self.__dac.vout_max))
            self.__dac.set_voltage_(limited_voltage)
            self._value = limited_voltage
