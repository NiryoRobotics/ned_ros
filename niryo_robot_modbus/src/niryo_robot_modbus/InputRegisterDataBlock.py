#!/usr/bin/env python

from pymodbus.datastore import ModbusSparseDataBlock

from .addressing.input_register_addressing import get_addressing


class InputRegisterDataBlock(ModbusSparseDataBlock):
    """
    Input Register: Read Only 16-bit data
    """

    def __init__(self, ros_wrapper):
        self.__addressing = get_addressing(ros_wrapper)
        super().__init__({address: False for address in self.__addressing.keys()})

    def getValues(self, address, count=1):
        if count <= 0:
            return []
        try:
            register_result = self.__addressing[address]()
        except (IndexError, KeyError) as error:
            raise ModbusIOException(str(error))

        return [register_result] + self.getValues(address + 1, count - 1)