# top_button.py
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

# Messages
from niryo_robot_arm_commander.msg import PausePlanExecution
from niryo_robot_status.msg import RobotStatus

from ..common.abstract_top_button import AbstractTopButton


class TopButton(AbstractTopButton):

    def __init__(self):
        super(TopButton, self).__init__()

        self.last_time_button_pressed = rospy.Time.from_sec(0)

        self.__last_is_prog_running = False
        self.__button_action_done = False

        self._robot_status = RobotStatus()
        rospy.Subscriber('/niryo_robot_status/robot_status', RobotStatus, self._callback_robot_status)

        # Seems to be overkill over that value due to reading IO time
        rospy.Timer(rospy.Duration.from_sec(1.0 / 10), self.check_button)

        rospy.loginfo("Niryo Button started")

    def _callback_robot_status(self, msg):
        self.__robot_status = msg
        # self._is_prog_running()

    def _is_prog_running(self):
        is_prog_running = self.__robot_status.robot_status in [
            self.__robot_status.RUNNING_AUTONOMOUS,
            self.__robot_status.PAUSE,
            self.__robot_status.LEARNING_MODE_AUTONOMOUS,
            self.__robot_status.LEARNING_MODE_AUTONOMOUS
        ]

        # if self.__last_is_prog_running != is_prog_running:
        #     self.__last_is_prog_running = is_prog_running

        #     if is_prog_running:
        #         self._send_pause_state(PausePlanExecution.PLAY)
        #     else:
        #         self._send_pause_state(PausePlanExecution.STANDBY)

        return is_prog_running

    def check_button(self, _event):
        button_was_pressed = self.is_button_pressed()
        self.read_value()

        if self.is_button_pressed():
            # Get press state
            if not button_was_pressed:
                self.__button_action_done = False
                self.last_time_button_pressed = rospy.Time.now()

            # Get long press to cancel program
            if not self.__button_action_done and (rospy.Time.now() - self.last_time_button_pressed).to_sec() > 2:
                self.__button_action_done = True

                if self._is_prog_running():
                    self._send_pause_state(PausePlanExecution.CANCEL)
                    rospy.logwarn("Button Manager - Cancel sequence")
                    self._cancel_program_from_program_manager()

        # Was pressed and is not anymore (release)
        elif not self.__button_action_done and button_was_pressed:
            elapsed_seconds = (rospy.Time.now() - self.last_time_button_pressed).to_sec()

            # Check if there is an action to do
            if 0.02 < elapsed_seconds < 2:
                self.__button_action_done = True
                if self._is_prog_running():
                    self._manage_python_program()
                else:
                    self._trigger_sequence_autorun()

    def _manage_python_program(self):
        # Pause the current move
        if self._pause_state in [PausePlanExecution.PLAY, PausePlanExecution.RESUME]:
            self._pause_program()
        elif self._pause_state == PausePlanExecution.PAUSE:
            self._resume_program()
