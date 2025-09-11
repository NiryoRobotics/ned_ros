# end_effector_io.py
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
from niryo_robot_utils import async_init

from niryo_robot_msgs.srv import SetBool
from end_effector_interface.srv import SetEEDigitalOut

from .io_objects import NiryoIO


class DigitalOutput(NiryoIO):
    def __init__(self, name):
        super(DigitalOutput, self).__init__(lock=None, pin=0, name=name)

        self.__set_ee_io_state_service = async_init.PromiseServiceProxy(
            "/niryo_robot_hardware_interface/end_effector_interface/set_ee_io_state",
            SetEEDigitalOut,
        )

        self._value = False

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        try:
            self.__set_ee_io_state_service(bool(value))
            self._value = bool(value)
        except rospy.service.ServiceException as e:
            rospy.logwarn(
                "End Effector Digital Output :: Failed to set its state\n{}".format(
                    str(e)
                )
            )

    def force_value(self, value):
        assert isinstance(value, (bool, int, float))
        self._value = bool(value)


class DigitalInput(NiryoIO):
    def __init__(self, name):
        super(DigitalInput, self).__init__(lock=None, pin=0, name=name)

        self.__set_ee_di_state_service = async_init.PromiseServiceProxy(
            "/niryo_robot_hardware_interface/end_effector_interface/set_ee_di_state",
            SetBool,
        )

        self._value = False

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        try:
            self.__set_ee_di_state_service(bool(value))
            self._value = bool(value)
        except rospy.service.ServiceException as e:
            rospy.logwarn(
                "End Effector Digital Input :: Failed to set its state\n{}".format(
                    str(e)
                )
            )
