# robot_rpi.py
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

from niryo_robot_status.msg import RobotStatus

from ..common.storage_manager import StorageManager
from .hardware.MCP23017 import MCP23017
from .top_button import TopButton
from .io_panel import IOPanel
from .fans_manager import FansManager
from .shutdown_manager import ShutdownManager
from .hotspot_button import HotspotButton
from .mcp_io_objects import McpIOManager


class RobotRpi:

    def __init__(self):
        self.__mcp = MCP23017(address=rospy.get_param("~mcp/address"), busnum=rospy.get_param("~mcp/i2c_bus"))

        self.__mcp_manager = McpIOManager(self.__mcp)

        self.__io_panel = IOPanel(mcp_manager=self.__mcp_manager)
        self.__fans_manager = FansManager(mcp_manager=self.__mcp_manager)

        self.__shutdown_manager = ShutdownManager(mcp_manager=self.__mcp_manager)

        self.__niryo_robot_button = TopButton()
        self.__hotspot_button = HotspotButton(mcp_manager=self.__mcp_manager)

        self.__storage_manager = StorageManager()

        rospy.on_shutdown(self.shutdown)

        self.robot_status_subscriber = rospy.Subscriber('/niryo_robot_status/robot_status',
                                                        RobotStatus,
                                                        self.__callback_robot_status,
                                                        queue_size=10)

    def __callback_robot_status(self, msg):
        if msg.robot_status <= RobotStatus.SHUTDOWN:
            self.shutdown()

    def shutdown(self):
        self.__fans_manager.shutdown()
        self.__hotspot_button.shutdown()
        self.__io_panel.shutdown()
        self.__mcp_manager.shutdown()
        rospy.sleep(2)
