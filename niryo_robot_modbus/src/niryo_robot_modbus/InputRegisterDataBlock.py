#!/usr/bin/env python

from pymodbus.datastore import ModbusSparseDataBlock


class InputRegisterDataBlock(ModbusSparseDataBlock):
    """
    Input Register: Read Only 16-bit data
    """

    def __init__(self):
        super().__init__([])

    def getValues(self, address, count=1):
        super().getValues(address, count)
