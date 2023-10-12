from abc import ABC, abstractmethod
from typing import Callable, Any, Optional, Dict, Tuple, List
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

import rospy


class WrapperAddress(ABC):

    def __init__(self, read: Optional[Callable] = None, write: Optional[Callable] = None, *args, **kwargs):
        """
        Initialize a WrapperAddress instance.

        :param read: The read callback function associated with this address.
        :type read: Callable
        :param write: The write callback function associated with this address.
        :type write: Callable, optional
        :param args: Additional positional arguments.
        :param kwargs: Additional keyword arguments.
        """
        self._read = read
        self._write = write
        self._size = 1

    @property
    def size(self):
        return self._size

    def is_readable(self) -> bool:
        """
        Check if a read function is implemented.

        :return: True if the object is readable, False otherwise.
        :rtype: bool
        """

        return self._read is not None

    def is_writable(self) -> bool:
        """
        Check if a write function is implemented.

        :return: True if the object is writable, False otherwise.
        :rtype: bool
        """

        return self._write is not None

    @abstractmethod
    def _encode(self, payload) -> List[Any]:
        raise NotImplementedError()

    @abstractmethod
    def _decode(self, payload: List[Any]):
        raise NotImplementedError()

    def read(self) -> Any:
        """
        Read data from this address using the associated read callback function.

        :return: The data read from this address.
        """
        return self._encode(self._read())

    def write(self, values) -> None:
        """
        Write data to this address using the associated write callback function.

        :param values: The data to be written to this address.
        :type values: Any
        :return: The result of the write operation, if applicable.
        """
        return self._write(self._decode(values))

    @classmethod
    def dynamic_addressing(cls,
                           offset: int,
                           count: int,
                           read: Optional[Callable] = None,
                           write: Optional[Callable] = None):
        """
        Generate a dictionary for dynamically addressing a callback with varying addresses.

        :param offset: The starting address value.
        :type offset: int
        :param count: The number of callback-address pairs to generate.
        :type count: int
        :param read: The read callback function to associate with each address.
                     It must take in parameter an int representing the current index - offset within the range
        :type read: Optional[Callable]
        :param write: The write callback function to associate with each address.
                      *args and **kwargs will be forwarded to it when called
        :type write: Optional[Callable]

        :returns: A dictionary where keys are integer addresses and values are WrapperAddress instances
                  associated with those addresses.
        :rtype: Dict[int, Callable]
        """
        wrapper_addresses = {}
        for address in range(offset, count + offset):
            read_cb = None if read is None else lambda ix=address - offset: read(ix)
            write_cb = None if write is None else lambda values, ix=address - offset: write(ix, values)
            wrapper_addresses[address] = cls(read_cb, write_cb)
        return wrapper_addresses


class DigitalWrapperAddress(WrapperAddress):

    def _encode(self, payload) -> List[bool]:
        return [bool(payload)]

    def _decode(self, payload: List):
        return [bool(value) for value in payload]


class AnalogWrapperAddress(WrapperAddress):
    __type_to_binary_payload_func: Dict[type, Tuple[Callable, Callable]] = {
        float: (BinaryPayloadBuilder.add_32bit_float, BinaryPayloadDecoder.decode_32bit_float),
        int: (BinaryPayloadBuilder.add_8bit_int, BinaryPayloadDecoder.decode_8bit_int),
        str: (BinaryPayloadBuilder.add_string, lambda decoder: BinaryPayloadDecoder.decode_string(decoder, 200)),
    }
    __type_size: Dict[type, int] = {
        float: 2,
        int: 1,
        str: 1,
    }

    def __init__(self, read: Optional[Callable] = None, write: Optional[Callable] = None, data_type=float):
        """
        Initialize a WrapperAddress instance.

        :param read: The read callback function associated with this address.
        :type read: Callable
        :param write: The write callback function associated with this address.
        :type write: Callable, optional
        :param data_type: the type this register will store. default: float
        :type data_type: type
        """
        super().__init__(read, write)
        self.__data_type = data_type
        if self.__data_type not in self.__type_to_binary_payload_func:
            rospy.logerr(f'Unsupported type "{type(self.__data_type)}" for payload "{self.__data_type}"')
            raise TypeError(f'Unsupported type "{type(self.__data_type)}" for payload "{self.__data_type}"')

        self._size = self.__type_size[self.__data_type]

    def _encode(self, payload) -> List[int]:
        builder_func = self.__type_to_binary_payload_func[self.__data_type][0]

        builder = BinaryPayloadBuilder()
        builder_func(builder, payload)
        return builder.to_registers()

    def _decode(self, payload: List[int]):
        decoder_func = self.__type_to_binary_payload_func[self.__data_type][0]
        decoder = BinaryPayloadDecoder(payload)
        return decoder_func(decoder)

    @classmethod
    def dynamic_addressing(cls,
                           offset: int,
                           count: int,
                           read: Optional[Callable] = None,
                           write: Optional[Callable] = None,
                           data_type=float):
        """
        Generate a dictionary for dynamically addressing a callback with varying addresses.

        :param offset: The starting address value.
        :type offset: int
        :param count: The number of callback-address pairs to generate.
        :type count: int
        :param read: The read callback function to associate with each address.
                     It must take in parameter an int representing the current index - offset within the range
        :type read: Optional[Callable]
        :param write: The write callback function to associate with each address.
                      *args and **kwargs will be forwarded to it when called
        :type write: Optional[Callable]
        :param data_type: the type of the data contained in the register address

        :returns: A dictionary where keys are integer addresses and values are WrapperAddress instances
                  associated with those addresses.
        :rtype: Dict[int, Callable]
        """
        wrapper_addresses = {}
        size = cls.__type_size[data_type]
        for address in range(offset, (count + offset) * size, size):
            ix = int((address - offset) / size)
            wrapper_addresses[address] = cls(
                lambda ix_=ix: read(ix_), lambda values, ix_=ix: write(ix_, values), data_type)
            for reserved_address in range(address + 1, address + size):
                wrapper_addresses[reserved_address] = cls()
        return wrapper_addresses
