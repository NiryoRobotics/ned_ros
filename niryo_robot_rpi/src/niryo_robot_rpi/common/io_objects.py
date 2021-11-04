# io_objects.py
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


class PinMode:
    """
    Pin Mode is either OUTPUT or INPUT
    """

    def __init__(self):
        pass

    DIGITAL_OUTPUT = 0
    DIGITAL_INPUT = 1

    ANALOG_OUTPUT = 2
    ANALOG_INPUT = 3


class NiryoIOException(Exception):
    pass


class NiryoIO(object):

    def __init__(self, lock, pin, name):
        self._lock = lock
        self._pin = pin
        self._name = name
        self._mode = None
        self._value = 0

    @property
    def mode(self):
        return self._mode

    @property
    def pin(self):
        return self._pin

    @property
    def name(self):
        return self._name

    def __str__(self):
        return self._name
