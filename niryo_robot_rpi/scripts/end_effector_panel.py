#!/usr/bin/env python

import rospy

from niryo_robot_rpi.rpi_ros_utils import activate_learning_mode, auto_calibration

# Messages
from std_msgs.msg import Int32, Bool
from end_effector_interface.msg import ButtonStatus
from niryo_robot_status.msg import RobotStatus


class NiryoEndEffectorPanel:
    def __init__(self):
        rospy.logdebug("Niryo end effector panel - Entering in Init")

        self._robot_status = RobotStatus()
        rospy.Subscriber('/niryo_robot_status/robot_status', RobotStatus, self._callback_robot_status)

        self.__learning_mode_button_state = ButtonStatus.NO_ACTION
        self.__learning_mode_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/free_drive_button_status',
            ButtonStatus, self.__callback_learning_mode_button)

        self.__save_pos_button_state = False
        self.__save_pos_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/save_pos_button_status',
            ButtonStatus, self.__callback_save_pos_button_status)

        self.__custom_button_state = ButtonStatus.NO_ACTION
        self.__custom_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/custom_button_status',
            ButtonStatus, self.__callback_custom_pos_button_status)

        self.__learning_mode_on = False
        self.__learning_mode_topic = rospy.Subscriber('/niryo_robot/learning_mode/state', Bool,
                                                      self.__callback_sub_learning_mode)

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

            if msg.action == ButtonStatus.HANDLE_HELD_ACTION:
                activate_learning_mode(True)
            elif msg.action == ButtonStatus.NO_ACTION and self.__learning_mode_button_state == ButtonStatus.HANDLE_HELD_ACTION:
                activate_learning_mode(False)
            elif msg.action == ButtonStatus.SINGLE_PUSH_ACTION:
                activate_learning_mode(not self.__learning_mode_on)

            self.__learning_mode_button_state = msg.action

    def __callback_save_pos_button_status(self, msg):
        if msg.action == ButtonStatus.NO_ACTION:
            pressed = False
        else:
            pressed = True

        if pressed != self.__save_pos_button_state:
            self.__save_pos_button_state = pressed
            if pressed:
                self.blockly_save_current_point()

    def __callback_custom_pos_button_status(self, msg):
        if self.__custom_button_state == ButtonStatus.HANDLE_HELD_ACTION and msg.action == ButtonStatus.NO_ACTION and self.__robot_status == RobotStatus.CALIBRATION_NEEDED:
            self.__custom_button_state = msg.action
            auto_calibration()
        else:
            self.__custom_button_state = msg.action

    def __callback_sub_learning_mode(self, msg):
        self.__learning_mode_on = msg.data

    def blockly_save_current_point(self):
        msg = Int32()
        msg.data = 1
        self.save_point_publisher.publish(msg)
