# rpi_io_objects.py
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

import time
import RPi.GPIO as GPIO

from ..common.io_objects import PinMode, NiryoIO


class DigitalPin(NiryoIO):

    def __init__(self, lock, pin, name, mode, state=False):
        super(DigitalPin, self).__init__(lock, pin, name)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self._mode = None
        self.mode = mode

        time.sleep(0.01)
        if self._mode == PinMode.DIGITAL_OUTPUT:
            self.value = state
        else:
            self._read()

    def _read(self):
        with self._lock:
            try:
                self._value = GPIO.input(self._pin)
            except RuntimeError:
                pass

    @property
    def value(self):
        if self._mode == PinMode.DIGITAL_INPUT:
            self._read()
        return self._value

    @value.setter
    def value(self, value):
        if self._mode == PinMode.DIGITAL_OUTPUT:
            with self._lock:
                self._value = bool(value)
                try:
                    GPIO.output(self._pin, GPIO.HIGH if self._value else GPIO.LOW)
                except RuntimeError:
                    pass

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        self._mode = mode
        if mode == PinMode.DIGITAL_OUTPUT:
            with self._lock:
                GPIO.setup(self._pin, GPIO.OUT)
            time.sleep(0.01)
            self.value = GPIO.LOW
        else:
            GPIO.setup(self._pin, GPIO.IN)
            time.sleep(0.01)
            self._read()


class Fan(DigitalPin):

    def __init__(self, lock, pin, name, temperature_on_threshold, temperature_off_threshold):
        super(Fan, self).__init__(lock, pin, name, PinMode.DIGITAL_OUTPUT, False)

        self.temperature_on_threshold = temperature_on_threshold
        self.temperature_off_threshold = temperature_off_threshold

    def update(self, temperature):
        if temperature >= self.temperature_on_threshold:
            self.value = 1
        elif temperature <= self.temperature_off_threshold:
            self.value = 0


class Led(DigitalPin):

    def __init__(self, lock, pin, name):
        super(Led, self).__init__(lock, pin, name, PinMode.DIGITAL_OUTPUT, False)

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0
