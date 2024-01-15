from __future__ import annotations
from abc import ABC, abstractmethod
from enum import Enum
from typing import List

import rospy

from conveyor_interface.msg import ConveyorFeedback

from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

from ..CommonStore import CommonStore
from ..util import safe_get, SupportedType


class ABCRegisterEntries(ABC):
    # - properties to overwrite
    data_type = None
    reserved_addresses = 0

    @classmethod
    def is_read_only(cls: ABCRegisterEntries) -> bool:
        return cls.set is ABCRegisterEntries.set

    @staticmethod
    @abstractmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        pass

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        self._ros_wrapper = ros_wrapper
        self._index = ix

    def __encode(self, value: SupportedType) -> List[int]:
        if self.data_type in [bool, int]:
            return [value]

        builder = BinaryPayloadBuilder()
        if self.data_type == float:
            builder.add_32bit_float(value)
        elif self.data_type == str:
            builder.add_string(value)
        return builder.to_registers()

    def read(self) -> List[int]:
        value = self.get()
        encoded_value = self.__encode(value)
        return encoded_value

    def __decode(self, value: List[int]) -> SupportedType:
        if self.data_type in [int, bool]:
            return value[0]

        decoder = BinaryPayloadDecoder(value)
        if self.data_type == float:
            return decoder.decode_32bit_float()
        elif self.data_type == str:
            return decoder.decode_string(200)
        else:
            raise TypeError(f"Unsupported data type: {self.data_type} for entry {self.__class__.__name__}")

    def write(self, value: List[int]) -> None:
        decoded_value = self.__decode(value)
        try:
            CommonStore.is_executing_command = True
            self.set(decoded_value)
            CommonStore.last_command_result = 0
        except Exception as exception:
            # TODO: use custom exceptions to handle more precisely the command result
            CommonStore.last_command_result = 1
            raise exception
        finally:
            CommonStore.is_executing_command = False

    @abstractmethod
    def get(self) -> SupportedType:
        pass

    def set(self, value: SupportedType):
        raise NotImplementedError()


class ABCRegisterEntry(ABCRegisterEntries, ABC):

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return 1

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        super().__init__(ros_wrapper, 0)


class ABCConveyorRegisterEntries(ABCRegisterEntries, ABC):
    reserved_addresses = 20

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


class ABCVisionRegisterEntry(ABCRegisterEntry, ABC):

    def _get_vision_target(self):
        target = self._ros_wrapper.detect_object(CommonStore.workspace_name,
                                                 CommonStore.target_shape,
                                                 CommonStore.target_color)
        return dict(zip(['found', 'rel_pose', 'shape', 'color'], target))


class ABCCommonStoreEntry(ABCRegisterEntry, ABC):
    common_store_attribute = ''

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        super().__init__(ros_wrapper)

    def check_exists(self):
        if not hasattr(CommonStore, self.common_store_attribute):
            raise ValueError(
                f'Common Store has no attribute "{self.common_store_attribute}" ({self.__class__.__name__})')

    def get(self) -> SupportedType:
        self.check_exists()
        attribute_value = getattr(CommonStore, self.common_store_attribute)
        if isinstance(attribute_value, Enum):
            return attribute_value.value
        return attribute_value

    def set(self, value: SupportedType) -> None:
        self.check_exists()
        attribute_value = getattr(CommonStore, self.common_store_attribute)
        if isinstance(attribute_value, Enum):
            # cast the value into an Enum instance
            value = attribute_value.__class__(value)
        setattr(CommonStore, self.common_store_attribute, value)


class ABCUserStoreEntries(ABCRegisterEntries):

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return 100

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        self.__internal_value = self.data_type()

    def get(self) -> SupportedType:
        return self.__internal_value

    def set(self, value: SupportedType) -> None:
        self.__internal_value = value
