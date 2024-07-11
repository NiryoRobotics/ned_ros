# fake_io_objects.py
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

from ...common.io_objects import PinMode, NiryoIO


class FakeDigitalIO(NiryoIO):
    def __init__(self, name, mode):
        super(FakeDigitalIO, self).__init__(None, None, name)

        assert mode in [PinMode.DIGITAL_OUTPUT, PinMode.DIGITAL_INPUT], \
            "The pin mode of a fake digital io must be either PinMode.ANALOG_INPUT or PinMode.ANALOG_OUTPUT"

        self._mode = mode
        self._value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        limited_value = max(0, min(int(value), 1))
        self._value = bool(limited_value)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        assert mode in [PinMode.DIGITAL_OUTPUT, PinMode.DIGITAL_INPUT], \
            "The pin mode of a fake digital io must be either PinMode.ANALOG_INPUT or PinMode.ANALOG_OUTPUT"

        if mode != self._mode:
            self._value = 0
            self._mode = mode
