#!/usr/bin/env python

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


from threading import Thread

from niryo_robot_rpi.common.io_objects import PinMode, NiryoIO, NiryoIOException

from .hardware.DACx0501 import DACx0501


class DigitalOutput(NiryoIO):
    def __init__(self, mcp, lock, pin, name, reversed=False):
        super(DigitalOutput, self).__init__(lock, pin, name)

        self._mcp = mcp
        self._mode = PinMode.DIGITAL_OUTPUT
        self._reversed = reversed

        with self._lock:
            self._mcp.enable_interrupt(self._pin, False)
            self._mcp.reverse_polarity(self._pin, False)
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
            self.value = 1
        elif temperature <= self.temperature_off_threshold:
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
            self._mcp.reverse_polarity(self._pin, True)

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

    def __init__(self, mcp, lock, pin, name):
        self.__on_press_function = None
        self.__on_release_function = None

        super(Button, self).__init__(mcp, lock, pin, name, reverse_polarity=False)
        with self._lock:
            self._mcp.pullup(self._pin, True)

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
    GAIN_TWO_THIRD = 0x00
    GAIN_ONE = 0x0200
    GAIN_TWO = 0x0400
    GAIN_FOUR = 0x0600
    GAIN_EIGHT = 0x0800
    GAIN_SIXTEEN = 0x0A00

    CONVERSION_TO_MV = {
        GAIN_TWO_THIRD: 0.1875,
        GAIN_ONE: 0.125,
        GAIN_TWO: 0.0625,
        GAIN_FOUR: 0.03125,
        GAIN_EIGHT: 0.015625,
        GAIN_SIXTEEN: 0.0078125,
    }

    def __init__(self, adc, lock, pin, name, reading_factor=1.0):
        super(AnalogInput, self).__init__(lock, pin, name)

        self.__adc = adc
        self.__mode = PinMode.ANALOG_INPUT
        self.__gain = self.GAIN_TWO_THIRD
        self.__reading_factor = reading_factor

    @property
    def gain(self):
        return self.__gain

    @gain.setter
    def gain(self, gain):
        assert isinstance(gain, (int, float))
        if gain not in [self.GAIN_TWO_THIRD, self.GAIN_ONE, self.GAIN_TWO, self.GAIN_FOUR, self.GAIN_EIGHT,
                        self.GAIN_SIXTEEN]:
            raise NiryoIOException("Gain must be in [AnalogInput.GAIN_TWO_THIRD, AnalogInput.GAIN_ONE, "
                                   "AnalogInput.GAIN_TWO, AnalogInput.GAIN_FOUR, AnalogInput.GAIN_EIGHT, "
                                   "AnalogInput.GAIN_SIXTEEN]")
        self.__gain = gain

    @property
    def value(self):
        with self._lock:
            try:
                self._value = round(self.__adc.read_adc(self._pin, gain=self.__gain) * (1.0 / self.__reading_factor) *
                                    self.CONVERSION_TO_MV[self.__gain] / 1000, 2)
            except TypeError:  # Handle bad behavior of the library ADS1115
                pass
            return self._value


class AnalogOutput(NiryoIO):

    def __init__(self, lock, pin, name, address, bus_i2c, v_ref, resolution):
        super(AnalogOutput, self).__init__(lock, pin, name)

        self.__dac = DACx0501(bus=bus_i2c, address=address, v_ref=v_ref, resolution=resolution)
        self.__mode = PinMode.ANALOG_OUTPUT

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (int, float))
        with self._lock:
            limited_voltage = max(0, min(value, self.__dac.v_ref))
            self.__dac.set_voltage_(limited_voltage)
            self._value = limited_voltage
