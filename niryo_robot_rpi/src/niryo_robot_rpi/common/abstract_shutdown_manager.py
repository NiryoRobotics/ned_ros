# abstract_shutdown_manager.py
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
import threading

from .rpi_ros_utils import send_reboot_command, send_shutdown_command

from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.srv import AdvertiseShutdown, AdvertiseShutdownRequest


class AbstractShutdownManager(object):
    def __init__(self, fake=False):
        rospy.logdebug("ShutdownManager - Entering in Init")
        self._fake = fake

        rospy.Service('~shutdown_rpi', AdvertiseShutdown, self.callback_shutdown_rpi)

        self._advertise_shutdown_service = rospy.ServiceProxy('/niryo_robot_status/advertise_shutdown',
                                                              AdvertiseShutdown)

    def callback_shutdown_rpi(self, req):
        if req.value == AdvertiseShutdownRequest.SHUTDOWN:
            self.request_shutdown()
            return CommandStatus.SUCCESS, 'Robot is shutting down'
        elif req.value == AdvertiseShutdownRequest.REBOOT:
            self.request_reboot()
            return CommandStatus.SUCCESS, 'Robot is rebooting'
        else:
            return CommandStatus.UNKNOWN_COMMAND, 'Incorrect value: 1 for shutdown, 2 for reboot'

    def request_shutdown(self):
        self.advertise_shutdown()
        send_shutdown_command_thread = threading.Timer(1.0, self.shutdown)
        send_shutdown_command_thread.start()

    def request_reboot(self):
        self.advertise_reboot()
        send_shutdown_command_thread = threading.Timer(1.0, self.reboot)
        send_shutdown_command_thread.start()

    def advertise_shutdown(self):
        try:
            rospy.wait_for_service('/niryo_robot_status/advertise_shutdown', timeout=0.2)
            self._advertise_shutdown_service.call(AdvertiseShutdownRequest.SHUTDOWN)
        except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException):
            pass

    def advertise_reboot(self):
        try:
            rospy.wait_for_service('/niryo_robot_status/advertise_shutdown', timeout=0.2)
            self._advertise_shutdown_service.call(AdvertiseShutdownRequest.REBOOT)
        except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException):
            pass

    def shutdown(self):
        print('Shutdown')
        if not self._fake:
            send_shutdown_command()

    def reboot(self):
        print('Reboot')
        if not self._fake:
            send_reboot_command()
