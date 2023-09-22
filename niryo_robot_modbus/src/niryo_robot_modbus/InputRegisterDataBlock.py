#!/usr/bin/env python

from pymodbus.datastore import ModbusSparseDataBlock

from .mapping import input_register_mapping


class InputRegisterDataBlock(ModbusSparseDataBlock):
    """
    Input Register: Read Only 16-bit data
    """

    def __init__(self, ros_wrapper):
        self.__ros_wrapper = ros_wrapper
        self.__mapping = input_register_mapping.get_mapping(self.__ros_wrapper)
        super().__init__(self.__mapping.keys())

    def getValues(self, address, count=1):
        super().getValues(address, count)
