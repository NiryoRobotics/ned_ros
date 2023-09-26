#!/usr/bin/env python3

from typing import List

from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.exceptions import ModbusIOException

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper

from .addressing.discrete_input_addressing import get_addressing


class DiscreteInputDataBlock(ModbusSparseDataBlock):
    """
    Discrete Input: Read Only binary data
    """

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self.__addressing = get_addressing(ros_wrapper)

        super().__init__({address: False for address in self.__addressing.keys()})

    # Override ModbusSparseDataBlock

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        if count <= 0:
            return []
        try:
            register_result = self.__addressing[address]()
        except (IndexError, KeyError) as error:
            raise ModbusIOException(str(error))

        return [register_result] + self.getValues(address + 1, count - 1)
