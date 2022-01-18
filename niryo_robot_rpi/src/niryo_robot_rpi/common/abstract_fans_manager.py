# abstract_fans_manager.py
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

from niryo_robot_msgs.msg import HardwareStatus


class AbstractFansManager(object):
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
            fan.value = False

    def _callback_hardware_status(self, msg):
        for fan in self._fans_list:
            fan.update(msg.rpi_temperature)
