#!/usr/bin/env python

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

from niryo_robot_msgs.msg import HardwareStatus


class FansManager(object):
    def __init__(self):
        self._fans_list = []

        self._hardware_status_subscriber = rospy.Subscriber(
            '/niryo_robot_hardware_interface/hardware_status', HardwareStatus, self._callback_hardware_status)

        rospy.on_shutdown(self.shutdown)

    def __del__(self):
        pass

    def shutdown(self):
        if self._hardware_status_subscriber is None:
            return

        try:
            self._hardware_status_subscriber.unregister()
            self._hardware_status_subscriber = None
        except AssertionError:
            return

        rospy.sleep(1)
        rospy.loginfo("Fan Manager - Stop fans")
        for fan in self._fans_list:
            rospy.loginfo(fan.name)
            fan.value = False

    def _callback_hardware_status(self, msg):
        for fan in self._fans_list:
            fan.update(msg.rpi_temperature)


class FansManagerNed2(FansManager):
    def __init__(self, mcp=None, lock=None):
        from niryo_robot_rpi.mcp_io_objects import Fan
        from niryo_robot_rpi.MCP23017 import MCP23017

        super(FansManagerNed2, self).__init__()
        if mcp is None:
            self.__mcp = MCP23017(address=rospy.get_param("~mcp/address"),
                                  busnum=rospy.get_param("~mcp/i2c_bus"))
        else:
            self.__mcp = mcp

        if lock is None:
            lock = Lock()

        self._fans_list = [Fan(self.__mcp, lock, fan["pin"], "FAN_{}".format(fan["pin"]),
                               fan["temperature_on_threshold"], fan["temperature_off_threshold"])
                           for fan in rospy.get_param("/niryo_robot_rpi/fans")]

        rospy.loginfo("Fan Manager - Started")


class FansManagerNedOne(FansManager):
    def __init__(self):
        from niryo_robot_rpi.rpi_io_objects import Fan

        super(FansManagerNedOne, self).__init__()

        lock = Lock()
        self._fans_list = [Fan(lock, fan["pin"], "FAN_{}".format(fan["pin"]),
                               fan["temperature_on_threshold"],
                               fan["temperature_off_threshold"])
                           for fan in rospy.get_param("/niryo_robot_rpi/fans")]

        rospy.loginfo("Fan Manager - Started")
