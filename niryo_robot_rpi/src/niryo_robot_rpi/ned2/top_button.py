#!/usr/bin/env python

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
import RPi.GPIO as GPIO
from threading import Thread

from niryo_robot_rpi.common.rpi_ros_utils import *

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from niryo_robot_arm_commander.msg import PausePlanExecution
from niryo_robot_status.msg import RobotStatus

# Services
from std_srvs.srv import Empty
from niryo_robot_msgs.srv import Trigger, SetInt


class TopButton:
    def __init__(self):

        rospy.logdebug("NiryoButton - Entering in Init")

        self.pin = rospy.get_param("~button/gpio")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        rospy.sleep(0.1)

        self.debug_loop_repetition = 5

        self.activated = True

        self.last_time_button_pressed = rospy.Time.from_sec(0)

        self._robot_status = RobotStatus()

        self._motor_debug_server_start = rospy.ServiceProxy('/niryo_robot_arm_commander/motor_debug_start', SetInt)
        self._motor_debug_server_stop = rospy.ServiceProxy('/niryo_robot_arm_commander/motor_debug_stop', Empty)
        self._motor_debug_thread = Thread()

        # - Publishers
        self.__pause_movement_publisher = rospy.Publisher('~pause_state',
                                                          PausePlanExecution, latch=True, queue_size=1)
        self._pause_state = None
        self.__last_is_prog_running = False
        self.__button_action_done = False
        self._send_pause_state(PausePlanExecution.STANDBY)

        rospy.Subscriber('/niryo_robot_status/robot_status', RobotStatus, self._callback_robot_status)

        # Seems to be overkill over that value due to reading IO time
        self.__button_state = self.read_value()
        rospy.Timer(rospy.Duration.from_sec(1.0 / 10), self.check_button)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Niryo Button started")

    def __del__(self):
        pass

    @staticmethod
    def shutdown():
        rospy.loginfo("Button Manager - Shutdown cleanup GPIO")
        rospy.sleep(0.5)
        GPIO.cleanup()

    def _is_button_pressed(self):
        return not self.__button_state

    def read_value(self):
        try:
            value = GPIO.input(self.pin)
        except RuntimeError:
            return False
        self.__button_state = value
        return self.__button_state

    def _callback_robot_status(self, msg):
        self.__robot_status = msg
        self._is_prog_running()

    def _is_prog_running(self):
        is_prog_running = self.__robot_status.robot_status in [self.__robot_status.RUNNING_AUTONOMOUS,
                                                               self.__robot_status.PAUSE,
                                                               self.__robot_status.LEARNING_MODE_AUTONOMOUS,
                                                               self.__robot_status.LEARNING_MODE_AUTONOMOUS]

        if self.__last_is_prog_running != is_prog_running:
            self.__last_is_prog_running = is_prog_running

            if is_prog_running:
                self._send_pause_state(PausePlanExecution.PLAY)
            else:
                self._send_pause_state(PausePlanExecution.STANDBY)

        return is_prog_running

    def check_button(self, _event):
        button_was_pressed = self._is_button_pressed()
        self.read_value()

        if self._is_button_pressed():
            # Get press state
            if not button_was_pressed:
                self.__button_action_done = False
                self.last_time_button_pressed = rospy.Time.now()

            # Get long press to cancle program
            if not self.__button_action_done and (rospy.Time.now() - self.last_time_button_pressed).to_sec() > 2:
                self.__button_action_done = True
                if self._motor_debug_thread.is_alive():
                    self._motor_debug_server_stop()
                    activate_learning_mode(True)
                elif self._is_prog_running():
                    self._send_pause_state(PausePlanExecution.CANCEL)
                    rospy.logwarn("Button Manager - Cancel sequence")
                    self._cancel_program_from_program_manager()
                    activate_learning_mode(True)

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

    def _trigger_sequence_autorun(self):
        rospy.loginfo("Button Manager - Run Auto-sequence")
        self._send_pause_state(PausePlanExecution.PLAY)

        if self._motor_debug_thread.is_alive():
            self._motor_debug_server_stop()
            self._motor_debug_thread.join()
        else:
            status, message = send_trigger_program_autorun()
            if status != CommandStatus.SUCCESS:
                self._trigger_motor_debug()

    def _trigger_motor_debug(self):
        try:
            rospy.wait_for_service("/niryo_robot_arm_commander/motor_debug_start", timeout=0.5)
            self._motor_debug_thread = Thread(target=self._motor_debug_server_start,
                                              name="motor_debug_button_thread",
                                              args=(self.debug_loop_repetition,))
            self._motor_debug_thread.start()
        except rospy.ROSException:
            pass

    @staticmethod
    def _cancel_program_from_program_manager():
        srv = rospy.ServiceProxy('/niryo_robot_programs_manager/stop_program', Trigger)
        resp = srv()
        return resp.status, resp.message

    def _resume_program(self):
        rospy.loginfo("Button Manager - Resume sequence")
        self._send_pause_state(PausePlanExecution.RESUME)

    def _pause_program(self):
        self._send_pause_state(PausePlanExecution.PAUSE)

    def _send_pause_state(self, state):
        self._pause_state = state
        self.__pause_movement_publisher.publish(self._pause_state)
