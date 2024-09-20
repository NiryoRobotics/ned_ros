from __future__ import annotations
from abc import ABC, abstractmethod
from enum import Enum
from typing import List
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

import rospy

from conveyor_interface.msg import ConveyorFeedback

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ConveyorID

from ..CommonStore import CommonStore
from ..util import (SupportedType,
                    modbus_exceptions_codes,
                    CHAR_PER_BLOCK,
                    MAX_STRING_LENGTH,
                    int_to_shape,
                    int_to_color,
                    shape_to_int,
                    color_to_int)


class ABCRegisterEntries(ABC):
    """
    Abstract base class for defining register entries.
    """
    data_type = None
    reserved_addresses = 0
    starting_address = 0

    @classmethod
    def is_read_only(cls: ABCRegisterEntries) -> bool:
        """
        Returns True if the register entry is read-only, False otherwise.
        """
        return cls.set is ABCRegisterEntries.set

    @staticmethod
    @abstractmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        """
        Returns the number of addresses occupied by the register entries.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :return: The number of addresses occupied by the register entry.
        :rtype: int
        """
        pass

    @classmethod
    def get_n_addresses(cls) -> int:
        if cls.data_type is None:
            raise ValueError("Data type can't be None")
        elif cls.data_type == float:
            return 2
        else:
            return 1

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        """
        Initializes a new instance of the ABCRegisterEntries class.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :param int ix: The index of the register entry.
        """
        self._ros_wrapper = ros_wrapper
        self._index = ix

    def __encode(self, value: SupportedType) -> List[int]:
        """
        Encodes the given value based on the data type.

        :param SupportedType value: The value to be encoded.
        :return: The encoded value.
        :rtype: List[int]
        """
        if self.data_type in [bool, int]:
            return [value]

        builder = BinaryPayloadBuilder()
        if self.data_type == float:
            builder.add_32bit_float(value)
        elif self.data_type == str:
            builder.add_string(value)
        return builder.to_registers()

    def read(self) -> List[int]:
        """
        Reads the value from the register entry and returns it in a format suitable for transmission.

        :return: The encoded value.
        :rtype: List[int]
        """
        value = self.get()
        encoded_value = self.__encode(value)
        return encoded_value

    def __decode(self, value: List[int]) -> SupportedType:
        """
        Decodes the given value into the appropriate data type.

        :param List[int] value: The value to be decoded.
        :return: The decoded value.
        :rtype: SupportedType
        """
        if self.data_type in [int, bool]:
            return value[0]

        decoder = BinaryPayloadDecoder.fromRegisters(value)
        if self.data_type == float:
            return decoder.decode_32bit_float()
        elif self.data_type == str:
            decoded_string = decoder.decode_string(MAX_STRING_LENGTH).decode()
            # a bit contains 2 char, so if there is an uneven number of char, the last one will be the NULL character
            decoded_string = decoded_string.replace('\x00', '')
            return decoded_string
        else:
            raise TypeError(f"Unsupported data type: {self.data_type} for entry {self.__class__.__name__}")

    def write(self, value: List[int]) -> None:
        """
        Writes the given value to the register entry.

        :param List[int] value: The value to be written.
        """
        decoded_value = self.__decode(value)
        try:
            CommonStore.is_executing_command = True
            self.set(decoded_value)
            CommonStore.last_command_result = 0
        except Exception as exception:
            # TODO: use custom exceptions to handle more precisely the command result
            try:
                CommonStore.last_command_result = modbus_exceptions_codes[exception.__class__]
            except KeyError:
                CommonStore.last_command_result = 1

            raise exception
        finally:
            CommonStore.is_executing_command = False

    @abstractmethod
    def get(self) -> SupportedType:
        """
        Retrieves the current value of the register entry.

        :return: The current value of the register entry.
        :rtype: SupportedType
        """
        pass

    def set(self, value: SupportedType):
        """
        Sets the value of the register entry.

        :param SupportedType value: The value to be set.
        """
        raise NotImplementedError()


class ABCRegisterEntry(ABCRegisterEntries, ABC):
    """
    Abstract base class for defining individual register entries.
    """

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        """
        Returns the number of addresses occupied by the register entry.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :return: The number of addresses occupied by the register entry.
        :rtype: int
        """
        return 1

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        """
        Initializes a new instance of the ABCRegisterEntry class.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        """
        super().__init__(ros_wrapper, 0)


class ABCConveyorRegisterEntries(ABCRegisterEntries, ABC):
    """
    Abstract base class for defining conveyor-related register entries.
    """
    reserved_addresses = 20

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        """
        Returns the number of addresses occupied by the register entry.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :return: The number of addresses occupied by the register entry.
        :rtype: int
        """
        can_conveyors = rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', [])
        ttl_conveyors = rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', [])
        n_conveyors = len(can_conveyors) + len(ttl_conveyors)
        return n_conveyors

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        """
        Initializes a new instance of the ABCConveyorRegisterEntries class.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :param int ix: The index of the register entry.
        """
        super().__init__(ros_wrapper, ix)

    @property
    def _conveyor_number(self) -> ConveyorID:
        """
        Returns the ros wrapper conveyor number
        :return: The conveyor number
        """
        try:
            return self._ros_wrapper.get_conveyors_number()[self._index]
        except IndexError:
            return ConveyorID.NONE

    def _safe_conveyor_feedback(self):
        """
        Safely retrieves conveyor feedback information.

        :return: Conveyor feedback information.
        :rtype: ConveyorFeedback
        """
        feedback = self._ros_wrapper.get_conveyors_feedback()
        try:
            return feedback[self._index]
        except (IndexError, KeyError):
            return ConveyorFeedback()

    def _safe_control_conveyor(self, bool_control_on=None, speed=None, direction=None):
        """
        Safely controls the conveyor.

        :param bool bool_control_on: Control status of the conveyor (True if on, False if off).
        :param float speed: Conveyor speed.
        :param float direction: Conveyor direction.
        """
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


class ABCVisionRegisterEntries(ABCRegisterEntries, ABC):
    """
    Abstract base class for defining vision-related register entries.
    """

    def __get_vision_target(self) -> dict:
        (found, pose, shape, color) = self._ros_wrapper.get_target_pose_from_cam(CommonStore.workspace_name,
                                                                                 CommonStore.height_offset,
                                                                                 int_to_shape[CommonStore.target_shape],
                                                                                 int_to_color[CommonStore.target_color])
        if found:
            pose_as_list = [
                pose.position.x, pose.position.y, pose.position.z, pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw
            ]
        else:
            pose_as_list = [0, 0, 0, 0, 0, 0]

        return {'found': found, 'pose': pose_as_list, 'shape': shape, 'color': color}

    def _is_target_found(self) -> bool:
        return self.__get_vision_target()['found']

    def _target_pose(self) -> list:
        return self.__get_vision_target()['pose']

    def _target_shape(self) -> int:
        return shape_to_int[self.__get_vision_target()['shape']]

    def _target_color(self) -> int:
        return color_to_int[self.__get_vision_target()['color']]


class ABCVisionRegisterEntry(ABCRegisterEntry, ABCVisionRegisterEntries, ABC):
    pass


class ABCCommonStoreEntry(ABCRegisterEntry, ABC):
    """
    Abstract base class for defining common store-related register entries.
    """
    common_store_attribute = ''

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        """
        Initializes a new instance of the ABCCommonStoreEntry class.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        """
        super().__init__(ros_wrapper)

    def check_exists(self):
        """
        Checks if the specified attribute exists in the CommonStore class.

        :raises ValueError: If the specified attribute does not exist in the CommonStore class.
        """
        if not hasattr(CommonStore, self.common_store_attribute):
            raise ValueError(
                f'Common Store has no attribute "{self.common_store_attribute}" ({self.__class__.__name__})')

    def get(self) -> SupportedType:
        """
        Retrieves the current value of the register entry from the CommonStore.

        :return: The current value of the register entry.
        :rtype: SupportedType
        """
        self.check_exists()
        attribute_value = getattr(CommonStore, self.common_store_attribute)
        if isinstance(attribute_value, Enum):
            return attribute_value.value
        return attribute_value

    def set(self, value: SupportedType) -> None:
        """
        Sets the value of the register entry in the CommonStore.

        :param SupportedType value: The value to be set.
        """
        self.check_exists()
        attribute_value = getattr(CommonStore, self.common_store_attribute)
        if isinstance(attribute_value, Enum):
            # cast the value into an Enum instance
            value = attribute_value.__class__(value)
        setattr(CommonStore, self.common_store_attribute, value)


class ABCUserStoreEntries(ABCRegisterEntries, ABC):
    """
    Abstract base class for defining user store-related register entries.
    """

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        """
        Returns the number of addresses occupied by the register entry.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :return: The number of addresses occupied by the register entry.
        :rtype: int
        """
        return 100

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        """
        Initializes a new instance of the ABCUserStoreEntries class.

        :param NiryoRosWrapper ros_wrapper: The ROS wrapper object.
        :param int ix: The index of the register entry.
        """
        super().__init__(ros_wrapper, ix)
        self.__internal_value = self.data_type()

    def get(self) -> SupportedType:
        """
        Retrieves the current value of the register entry.

        :return: The current value of the register entry.
        :rtype: SupportedType
        """
        return self.__internal_value

    def set(self, value: SupportedType) -> None:
        """
        Sets the value of the register entry.

        :param SupportedType value: The value to be set.
        """
        self.__internal_value = value


class ABCStringEntries(ABCRegisterEntries, ABC):

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix * CHAR_PER_BLOCK)
        self._upper_index = self._index + CHAR_PER_BLOCK
