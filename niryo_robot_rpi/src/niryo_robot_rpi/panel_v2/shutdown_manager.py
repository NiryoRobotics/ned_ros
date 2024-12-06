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
import actionlib
import rospy
from threading import Event, Timer

from niryo_robot_programs_manager_v2.msg import ExecuteProgramAction

from std_msgs.msg import String
from niryo_robot_msgs.srv import AdvertiseShutdownRequest
from niryo_robot_msgs.srv import Trigger

from ..common.gpio_expander_adapters import GpioManager
from ..common.rpi_ros_utils import send_shutdown_command, send_reboot_command
from ..common.abstract_shutdown_manager import AbstractShutdownManager
from .mcp_io_objects import McpIOManager


class ShutdownManager(AbstractShutdownManager):
    SHUTDOWN_TIMEOUT = 10

    def __init__(self, mcp_manager=None, fake=False):
        super(ShutdownManager, self).__init__(fake=fake)

        try:
            self.__turn_off_sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/turn_off_sound")
        except KeyError:
            self.__turn_off_sound_name = ""

        self.__shutdown_requested = False
        self.__shutdown_event = Event()
        self.__sound_status_sub = None
        self.__current_sound = ""

        pins_type = rospy.get_param("~shutdown_manager/pins_type")

        if pins_type == 'mcp' and mcp_manager is not None:
            self.__manager = mcp_manager
        elif pins_type == 'mcp':
            self.__manager = McpIOManager()
        elif pins_type == 'gpio':
            self.__manager = GpioManager()

        self.__shutdown_output = self.__manager.add_output(rospy.get_param("~shutdown_manager/digital_output"),
                                                           "shutdown_output")
        self.__shutdown_input = self.__manager.add_button(rospy.get_param("~shutdown_manager/digital_input"),
                                                          "shutdown_input")

        self.__shutdown_input.on_press(self.request_shutdown)

    def request_shutdown(self):
        if not self.__shutdown_requested:
            self.__shutdown_requested = True
            self.__shutdown_output.value = True
            self._advertise_shutdown_service.call(AdvertiseShutdownRequest.SHUTDOWN)
            send_shutdown_command_thread = Timer(1.0, self.shutdown)
            send_shutdown_command_thread.start()

    def shutdown(self):
        self.__shutdown_output.value = False
        self.__shutdown_input.disable_on_press()
        self.__stop_move()
        self.wait_end_of_sound()
        if not self._fake:
            send_shutdown_command(learning_mode=False)

    def reboot(self):
        self.__stop_move()
        self.wait_end_of_sound()
        if not self._fake:
            send_reboot_command(learning_mode=False)

    @staticmethod
    def __stop_move():
        """
        Stop the robot movement
        """
        try:
            stop_move_service = rospy.ServiceProxy('/niryo_robot_arm_commander/stop_command', Trigger)
            stop_move_service()
        except (rospy.ServiceException, rospy.ROSInterruptException, rospy.ROSException):
            pass

        try:
            action_client = actionlib.SimpleActionClient('/niryo_robot_programs_manager_v2/execute_program',
                                                         ExecuteProgramAction)
            action_client.cancel_all_goals()
        except (rospy.ServiceException, rospy.ROSInterruptException, rospy.ROSException):
            pass

    def wait_end_of_sound(self):
        self.__shutdown_event.clear()
        self.__sound_status_sub = rospy.Subscriber("/niryo_robot_sound/sound", String, self.__callback_sound_state)
        self.__shutdown_event.wait(self.SHUTDOWN_TIMEOUT)

    def __callback_sound_state(self, msg):
        if self.__current_sound == self.__turn_off_sound_name and msg.data == "":
            self.__shutdown_event.set()
        self.__current_sound = msg.data
