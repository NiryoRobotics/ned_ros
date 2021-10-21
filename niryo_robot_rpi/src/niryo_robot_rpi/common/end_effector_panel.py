#!/usr/bin/env python

# end_effector_panel.py.py
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

from niryo_robot_rpi.common.rpi_ros_utils import activate_learning_mode, auto_calibration

from .end_effector_io import DigitalInput, DigitalOutput

# Messages
from std_msgs.msg import Int32, Bool
from end_effector_interface.msg import EEButtonStatus
from niryo_robot_status.msg import RobotStatus
from end_effector_interface.msg import EEIOState


class NiryoEndEffectorPanel:
    def __init__(self):
        rospy.logdebug("Niryo end effector panel - Entering in Init")

        # - Init
        self._robot_status = RobotStatus()

        self.__learning_mode_button_state = EEButtonStatus.NO_ACTION
        self.__custom_button_state = EEButtonStatus.NO_ACTION
        self.__save_pos_button_state = False

        self.__learning_mode_on = False

        self.digital_input = DigitalInput(rospy.get_param("~end_effector_ios/digital_input"))
        self.digital_output = DigitalOutput(rospy.get_param("~end_effector_ios/digital_output"))

        # - Subscribers
        rospy.Subscriber('/niryo_robot_status/robot_status', RobotStatus, self._callback_robot_status)

        self.__learning_mode_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/free_drive_button_status',
            EEButtonStatus, self.__callback_learning_mode_button)

        self.__save_pos_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/save_pos_button_status',
            EEButtonStatus, self.__callback_save_pos_button_status)

        self.__custom_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/custom_button_status',
            EEButtonStatus, self.__callback_custom_pos_button_status)

        self.__learning_mode_topic = rospy.Subscriber('/niryo_robot/learning_mode/state', Bool,
                                                      self.__callback_sub_learning_mode)

        self.__ee_io_state_topic = rospy.Subscriber('/niryo_robot_hardware_interface/end_effector_interface/io_state',
                                                    EEIOState, self.__callback_ee_io_state)

        # - Publishers
        self.save_point_publisher = rospy.Publisher(
            "/niryo_robot/blockly/save_current_point", Int32, queue_size=10)

        self.__button_state_publisher = rospy.Publisher(
            "/niryo_robot/rpi/is_button_pressed", Bool, latch=True, queue_size=1)

        rospy.loginfo("Niryo end effector panel started")

        rospy.on_shutdown(self.on_shutdown)

    def __del__(self):
        pass

    def on_shutdown(self):
        self.__learning_mode_button_topic.unregister()
        self.__save_pos_button_topic.unregister()
        self.__learning_mode_topic.unregister()

    def _callback_robot_status(self, msg):
        self.__robot_status = msg.robot_status

    def __callback_learning_mode_button(self, msg):
        if self.__learning_mode_button_state != msg:

            if msg.action == EEButtonStatus.HANDLE_HELD_ACTION:
                activate_learning_mode(True)
            elif (msg.action == EEButtonStatus.NO_ACTION and
                  self.__learning_mode_button_state == EEButtonStatus.HANDLE_HELD_ACTION):
                activate_learning_mode(False)

            self.__learning_mode_button_state = msg.action

    def __callback_save_pos_button_status(self, msg):
        if msg.action in [EEButtonStatus.NO_ACTION]:
            pressed = False
        elif msg.action in [EEButtonStatus.SINGLE_PUSH_ACTION, EEButtonStatus.LONG_PUSH_ACTION]:
            pressed = True
        else:
            return

        if pressed != self.__save_pos_button_state:
            self.__save_pos_button_state = pressed
            if pressed:
                self.blockly_save_current_point()

    def __callback_custom_pos_button_status(self, msg):
        if (self.__custom_button_state != EEButtonStatus.NO_ACTION and
                msg.action == EEButtonStatus.NO_ACTION and self.__robot_status == RobotStatus.CALIBRATION_NEEDED):
            self.__custom_button_state = msg.action
            auto_calibration()
        else:
            self.__custom_button_state = msg.action

    def __callback_sub_learning_mode(self, msg):
        self.__learning_mode_on = msg.data

    def __callback_ee_io_state(self, msg):
        self.digital_input.value = msg.digital_input
        self.digital_output.value = msg.digital_output

    def blockly_save_current_point(self):
        msg = Int32()
        msg.data = 1
        self.save_point_publisher.publish(msg)
