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
import actionlib
import rospy
import RPi.GPIO as GPIO

from niryo_robot_rpi.common.rpi_ros_utils import send_trigger_program_autorun

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from niryo_robot_arm_commander.msg import PausePlanExecution

# Services
from niryo_robot_msgs.srv import Trigger, SetInt
from niryo_robot_programs_manager_v2.msg import ExecuteProgramAction


class AbstractTopButton(object):

    def __init__(self):
        rospy.logdebug("NiryoButton - Entering in Init")

        self.pin = rospy.get_param("~button/gpio")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        rospy.sleep(0.1)

        rospy.Service("/niryo_robot/rpi/change_button_mode", SetInt, self.callback_change_button_mode)

        # - Publishers
        self._pause_state = None
        self._pause_movement_publisher = rospy.Publisher('~pause_state', PausePlanExecution, latch=True, queue_size=1)
        self._send_pause_state(PausePlanExecution.STANDBY)

        self._button_state = None
        self._button_state = self.read_value()

        rospy.on_shutdown(self.shutdown)

    def __del__(self):
        pass

    def read_value(self):
        try:
            value = GPIO.input(self.pin)
        except RuntimeError:
            return False
        self._button_state = value

        return self._button_state

    @staticmethod
    def shutdown():
        rospy.loginfo("Button Manager - Shutdown cleanup GPIO")
        rospy.sleep(0.5)
        GPIO.cleanup()

    @staticmethod
    def callback_change_button_mode(_req):
        return {"status": CommandStatus.SUCCESS, "message": ""}

    def is_button_pressed(self):
        return not self._button_state

    def _trigger_sequence_autorun(self):
        rospy.loginfo("Button Manager - Run Auto-sequence")
        self._send_pause_state(PausePlanExecution.PLAY)
        _status, _message = send_trigger_program_autorun()

    @staticmethod
    def _cancel_program_from_program_manager():
        action_client = actionlib.SimpleActionClient('/niryo_robot_programs_manager_v2/execute_program',
                                                     ExecuteProgramAction)
        action_client.cancel_all_goals()

    def _resume_program(self):
        rospy.loginfo("Button Manager - Resume sequence")
        self._send_pause_state(PausePlanExecution.RESUME)

    def _pause_program(self):
        self._send_pause_state(PausePlanExecution.PAUSE)

    def _send_pause_state(self, state):
        self._pause_state = state
        self._pause_movement_publisher.publish(self._pause_state)
