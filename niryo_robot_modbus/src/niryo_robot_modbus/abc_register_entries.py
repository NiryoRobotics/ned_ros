from __future__ import annotations
from abc import ABC, abstractmethod
import rospy

from conveyor_interface.msg import ConveyorFeedback

from niryo_robot_python_ros_wrapper import NiryoRosWrapper

from .util import safe_get


class ABCRegisterEntry(ABC):
    starting_address = -1
    data_type = None

    @classmethod
    def is_read_only(cls: ABCRegisterEntry) -> bool:
        return getattr(cls.write, '__isabstractmethod__', False)

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self._ros_wrapper = ros_wrapper

    @property
    def address(self) -> int:
        return self.starting_address

    @abstractmethod
    def read(self):
        pass

    @abstractmethod
    def write(self, value):
        pass


class ABCRegisterEntries(ABCRegisterEntry, ABC):

    @staticmethod
    @abstractmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        pass

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper)
        self._index = ix

    @property
    def address(self) -> int:
        return self.starting_address + self._index


class ABCConveyorRegisterEntries(ABCRegisterEntries, ABC):

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        can_conveyors = rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', [])
        ttl_conveyors = rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', [])
        n_conveyors = len(can_conveyors) + len(ttl_conveyors)
        return n_conveyors

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)

    def _safe_conveyor_feedback(self):
        return safe_get(self._ros_wrapper.get_conveyors_feedback(), self._index, ConveyorFeedback())

    def _safe_control_conveyor(self, bool_control_on=None, speed=None, direction=None):
        conveyors_id = self._ros_wrapper.get_conveyors_number()
        if len(conveyors_id) <= self._index:
            return

        feedback = self._ros_wrapper.get_conveyors_feedback()[self._index]
        if bool_control_on is None:
            bool_control_on = feedback.running
        if speed is None:
            speed = feedback.speed
        if direction is None:
            direction = feedback.direction

        self._ros_wrapper.control_conveyor(conveyors_id[self._index], bool_control_on, speed, direction)
