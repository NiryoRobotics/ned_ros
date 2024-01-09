from __future__ import annotations
from enum import Enum, auto
from typing import List, Callable, Dict, Tuple, Union, Any, Iterable
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

from . import logger

# - Custom types - #

SupportedType = Union[bool, float, int, str]
ReadCallback = Callable[[], SupportedType]
WriteCallback = Callable[[SupportedType], Any]

MultipleReadCallback = Callable[[int], SupportedType]
MultipleWriteCallback = Callable[[int, SupportedType], None]

# - Utility classes - #


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
        function_code = function_code % 10
        for register_type in cls:
            if function_code in register_type.value:
                return register_type
        raise ValueError(f'"{function_code}" is not an implemented function code')


class MoveType(Enum):
    MOVE_JOINT = auto()
    MOVE_POSE = auto()
    MOVE_LINEAR = auto()


class PayloadHandler:
    __MAX_STRING_LENGTH = 200
    __type_to_binary_payload_func: Dict[type, Tuple[Callable, Callable]] = {
        float: (BinaryPayloadBuilder.add_32bit_float, BinaryPayloadDecoder.decode_32bit_float),
        int: (BinaryPayloadBuilder.add_8bit_int, BinaryPayloadDecoder.decode_8bit_int),
        str: (BinaryPayloadBuilder.add_string,
              lambda decoder: BinaryPayloadDecoder.decode_string(decoder, PayloadHandler.__MAX_STRING_LENGTH)),
    }
    __type_size: Dict[type, int] = {
        float: 2,
        int: 1,
        str: 1,
    }

    def encode(self, payload: SupportedType, data_type: type) -> List[int]:
        logger.info(f'payload to encode: {payload}')
        if data_type == bool:
            return [int(payload)]

        builder_func = self.__type_to_binary_payload_func[data_type][0]

        builder = BinaryPayloadBuilder()
        builder_func(builder, payload)
        logger.info(f'encoded payload: {builder.to_registers()}')
        return builder.to_registers()

    def decode(self, payload: List[int], data_type: type) -> SupportedType:
        logger.info(f'payload to decode: {payload} ({data_type})')
        if data_type == bool:
            logger.info(f'decoded payload: {bool(payload[0])}')
            return bool(payload[0])
        decoder_func = self.__type_to_binary_payload_func[data_type][0]
        decoder = BinaryPayloadDecoder(payload)
        logger.info(f'decoded payload: {decoder_func(decoder)}')
        return decoder_func(decoder)


# - Utility functions - #


def safe_get(iterable: Iterable, item: Any, default_value: Any) -> Any:
    """
    Safely retrieve an element in an iterable object, else return  a default value

    :param iterable: an iterable
    :param item: the key to access the element in iterable
    :param default_value: The default value to be returned if the item is not found
    :return: the got item or the default value if not found
    """
    try:
        return iterable[item]
    except (IndexError, KeyError):
        return default_value
