# fans_manager.py
# Copyright (C) 2017 Niryo
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

from ..common.abstract_fans_manager import AbstractFansManager
from .rpi_io_objects import Fan


class FansManager(AbstractFansManager):

    def __init__(self):
        super(FansManager, self).__init__()

        lock = Lock()
        self._fans_list = [
            Fan(lock,
                fan["pin"],
                "FAN_{}".format(fan["pin"]),
                fan["temperature_on_threshold"],
                fan["temperature_off_threshold"]) for fan in rospy.get_param("/niryo_robot_rpi/fans")
        ]

        rospy.loginfo("Fan Manager - Started")
