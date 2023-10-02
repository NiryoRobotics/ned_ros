from abc import ABC, abstractmethod
from typing import Callable, Any


class WrapperAddress(ABC):

    def __init__(self, read: Callable, write: Callable = None, *args, **kwargs):
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

    def read(self) -> Any:
        """
        Read data from this address using the associated read callback function.

        :return: The data read from this address.
        """
        return self._read()

    def write(self, values) -> None:
        """
        Write data to this address using the associated write callback function.

        :param values: The data to be written to this address.
        :type values: Any
        :return: The result of the write operation, if applicable.
        """
        return self._write(values)

    @classmethod
    def dynamic_addressing(cls,
                           offset: int,
                           count: int,
                           read: Callable,
                           write: Callable = None,
                           step=1,
                           *args,
                           **kwargs):
        """
        Generate a dictionary for dynamically addressing a callback with varying addresses.

        :param offset: The starting address value.
        :type offset: int
        :param count: The number of callback-address pairs to generate.
        :type count: int
        :param read: The read callback function to associate with each address.
                     It must take in parameter an int representing the current index - offset within the range
        :type read: (int) -> bool
        :param write: The write callback function to associate with each address.
                      *args and **kwargs will be forwarded to it when called
        :type write: Callable
        :param step: The step size between consecutive addresses. Defaults to 1.
        :type step: int, optional

        :returns: A dictionary where keys are integer addresses and values are WrapperAddress instances
                  associated with those addresses.
        :rtype: Dict[int, Callable]
        """
        wrapper_addresses = {}
        for address in range(offset, (step * count) + offset, step):
            wrapper_addresses[address] = cls(lambda ix=address - offset: read(ix),
                                             (lambda values, ix=address - offset: write(ix, values)),
                                             *args,
                                             **kwargs)
        return wrapper_addresses


class DigitalWrapperAddress(WrapperAddress):

    def read(self) -> bool:
        """
        Read a digital value from this address using the associated read callback function.

        :return: The digital value read from this address as a boolean.
        :rtype: bool
        """
        callback_result = super().read()
        return bool(callback_result)


class AnalogWrapperAddress(WrapperAddress):
    __SIGN_MASK = 0b1000000000000000

    def __init__(self, read: Callable, write: Callable = None, precision=0):
        """
        Initialize an AnalogWrapperAddress instance for handling analog values.

        :param read: The read callback function associated with this address.
        :type read: Callable
        :param write: The write callback function associated with this address.
        :type write: Callable, optional
        :param precision: The number of decimals to keep when encoding values. It will also be used to decode them.
                          Defaults to 0 (no decimal places).
        :type precision: int, optional
        """
        self._precision = precision
        super().__init__(read, write)

    def _encode(self, value: float) -> int:
        """
        Encode an analog value as an integer representing a 16 bits array.

        It uses the 15 first bits to encode the value, and the 16th bit to determine the sign of the value.

        :param value: The analog value to encode.
        :type value: float
        :return: The encoded value.
        :rtype: int
        """
        rounded_value = round(value * 10**self._precision)
        if rounded_value < 0:
            rounded_value = self.__SIGN_MASK | abs(rounded_value)

        return rounded_value

    def _decode(self, value: int) -> float:
        """
        Decode an encoded analog value back to its original float representation.

        :param value: The encoded integer value to decode.
        :type value: int
        :return: The decoded analog value as a float.
        :rtype: float
        """
        if self.__SIGN_MASK & value != 0:
            value -= self.__SIGN_MASK
        return value / 10**self._precision

    def read(self) -> int:
        """
        Read an encoded analog value from this address using the associated read callback function.

        :return: The encoded analog value as an integer.
        :rtype: int
        """
        return self._encode(super().read())

    def write(self, values) -> None:
        """
        Write encoded analog values to this address using the associated write callback function.

        :param values: The encoded analog values to be written.
        :type values: List[int]
        """
        super().write([self._decode(value) for value in values])
