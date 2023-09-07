#!/usr/bin/env python

from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock


class SequentialDataBlock(ModbusSequentialDataBlock):

    def __init__(self, address=0, values=[0] * 1000):
        super().__init__(address, values)

        # Called from internal functions
        # Modbus addresses start at 1
        # There is an offset with what the client is asking

    def setValuesOffset(self, address, values):
        self.setValues(address + 1, values)

    def getValuesOffset(self, address, count=1):
        return self.getValues(address + 1, count)


class SparseDataBlock(ModbusSparseDataBlock):

    def __init__(self, values=[0] * 1000):
        super().__init__(values=values)

    # Called from internal functions
    # Modbus addresses start at 1
    # There is an offset with what the client is asking
    def setValuesOffset(self, address, values):
        self.setValues(address + 1, values)

    def getValuesOffset(self, address, count=1):
        return self.getValues(address + 1, count)
