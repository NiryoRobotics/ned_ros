from abc import ABC, abstractmethod
from typing import List, Dict
import rospy
import traceback

from pymodbus.datastore import ModbusSparseDataBlock
from conveyor_interface.msg import ConveyorFeedback

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ConveyorID

from .WrapperAddress import WrapperAddress


class WrapperDataBlock(ABC, ModbusSparseDataBlock):

    def __init__(self, ros_wrapper: NiryoRosWrapper, shared_data):
        self._ros_wrapper = ros_wrapper
        self._shared_data = shared_data
        self._addressing = self._get_addressing()
        super().__init__({a: 0 for a in self._addressing.keys()})

    @abstractmethod
    def _get_addressing(self) -> Dict[int, WrapperAddress]:
        raise NotImplementedError()

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        self._shared_data.is_executing_command = True
        try:
            if count <= 0:
                return []

            register_entry = self._addressing[address]
            if register_entry.is_readable():
                return register_entry.read() + self.getValues(address + register_entry.size,
                                                              count - register_entry.size)

            self._shared_data.is_executing_command = False
            return super().getValues(address, count)
        except Exception as e:
            rospy.logerr(e)
            traceback.print_exc()
            self._shared_data.is_executing_command = False

    def setValues(self, address: int, values: List[bool]) -> None:
        self._shared_data.is_executing_command = True
        register_entry = self._addressing[address]
        if register_entry.is_writable():
            return register_entry.write(values)

        super().setValues(address, values)
        self._shared_data.is_executing_command = False

    # utilities functions used by children

    def _safe_get(self, iterable, item, default_value):
        """
        Safely retrieve an element in an iterable object, else return  a default value

        :param iterable: an iterable
        :type iterable: Any class implementing __getattribute__
        :param item: the key to access the element in iterable
        :type item: Any
        :param default_value: The default value to be returned if the item is not found
        :type default_value: Any
        :return: the got item or the default value if not found
        :rtype: Any
        """
        try:
            return iterable[item]
        except (IndexError, KeyError):
            return default_value

    def __attach_all_conveyors(self):
        """
        Try to set all the connected conveyors which are not set yet
        """
        result = ConveyorID.NONE
        while result.value != ConveyorID.NONE.value:
            result = self._ros_wrapper.set_conveyor()
            rospy.loginfo(result)

    def _safe_conveyor_feedback(self, ix):
        """
        Safely retrieve the conveyor feedback at index ``ix``.
        If the element is not in the list, returns an empty ``ConveyorFeedback`` object.

        :param ix: the index of the conveyor
        :type ix: int
        :return: the conveyor feedback or an empty conveyor feedback if not found
        :rtype: ConveyorFeedback
        """
        self.__attach_all_conveyors()
        return self._safe_get(self._ros_wrapper.get_conveyors_feedback(), ix, ConveyorFeedback())

    def _safe_control_conveyor(self, ix, bool_control_on=None, speed=None, direction=None):
        self.__attach_all_conveyors()
        conveyors_id = self._ros_wrapper.get_conveyors_number()
        if len(conveyors_id) <= ix:
            return

        feedback = self._ros_wrapper.get_conveyors_feedback()[ix]
        if bool_control_on is None:
            bool_control_on = feedback.running
        if speed is None:
            speed = feedback.speed
        if direction is None:
            direction = feedback.direction

        self._ros_wrapper.control_conveyor(conveyors_id[ix], bool_control_on, speed, direction)

    def _get_vision_target(self):
        target = self._ros_wrapper.detect_object(self._shared_data.workspace_name,
                                                 self._shared_data.shape,
                                                 self._shared_data.color)
        return dict(zip(['found', 'rel_pose', 'shape', 'color'], target))
