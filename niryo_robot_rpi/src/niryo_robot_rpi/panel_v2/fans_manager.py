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

from ..common.abstract_fans_manager import AbstractFansManager
from .mcp_io_objects import McpIOManager


class FansManager(AbstractFansManager):

    def __init__(self, mcp_manager):
        super(FansManager, self).__init__()

        self.__mcp_manager = mcp_manager if mcp_manager is not None else McpIOManager()

        self._fans_list = [
            self.__mcp_manager.add_fan(fan["pin"],
                                       "FAN_{}".format(fan["pin"]),
                                       fan["temperature_on_threshold"],
                                       fan["temperature_off_threshold"])
            for fan in rospy.get_param("/niryo_robot_rpi/fans")
        ]

        rospy.loginfo("Fan Manager - Started")
