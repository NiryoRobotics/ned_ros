from __future__ import annotations

from enum import Enum
from typing import Union

from niryo_robot_python_ros_wrapper import ObjectShape, ObjectColor

from niryo_robot_utils import NiryoRosWrapperException

# - Constants - #

MAX_STRING_LENGTH = 40
N_BITS_PER_CHAR = 8
BLOCK_SIZE = 16
CHAR_PER_BLOCK = BLOCK_SIZE // N_BITS_PER_CHAR

SupportedType = Union[bool, float, int, str]

int_to_shape = {0: ObjectShape.ANY, 1: ObjectShape.CIRCLE, 2: ObjectShape.SQUARE}
shape_to_int = {shape: i for i, shape in int_to_shape.items()}
int_to_color = {0: ObjectColor.ANY, 1: ObjectColor.RED, 2: ObjectColor.GREEN, 3: ObjectColor.BLUE}
color_to_int = {color: i for i, color in int_to_color.items()}


class ModbusException(Exception):
    pass


modbus_exceptions_codes = {
    ModbusException: 1,
    NiryoRosWrapperException: 2,
}


class RegisterType(Enum):
    """
    Enumeration of modbus registers using their read / write functions codes as keys
    """
    COIL = (0x01, 0x05)
    DISCRETE_INPUT = (0x02, )
    HOLDING_REGISTER = (0x03, 0X06)
    INPUT_REGISTER = (0x04, )

    def is_read_only(self) -> bool:
        return len(self.value) == 1

    @classmethod
    def from_function_code(cls, function_code: int) -> RegisterType:
        for register_type in cls:
            if function_code in register_type.value:
                return register_type
        raise ValueError(f'"{function_code}" is not an implemented function code')


class MoveType(Enum):
    MOVE_JOINT = 0
    MOVE_POSE = 1
    MOVE_LINEAR = 2
