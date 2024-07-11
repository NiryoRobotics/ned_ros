# shutdown_manager.py
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

from ..common.abstract_shutdown_manager import AbstractShutdownManager
from ..common.rpi_ros_utils import send_led_state, LedState


class ShutdownManager(AbstractShutdownManager):

    def __init__(self, fake=False):
        super(ShutdownManager, self).__init__(fake=fake)

    def shutdown(self):
        send_led_state(LedState.SHUTDOWN)
        super(ShutdownManager, self).shutdown()

    def reboot(self):
        send_led_state(LedState.SHUTDOWN)
        super(ShutdownManager, self).reboot()
