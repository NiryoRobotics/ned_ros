from abc import ABC, abstractmethod
from typing import List, Dict

from pymodbus.datastore import ModbusSparseDataBlock

import rospy
from .WrapperAddress import WrapperAddress
import traceback


class WrapperDataBlock(ABC, ModbusSparseDataBlock):

    def __init__(self, ros_wrapper):
        self._ros_wrapper = ros_wrapper
        self._addressing = self._get_addressing()
        super().__init__({a: 0 for a in self._addressing.keys()})

    @abstractmethod
    def _get_addressing(self) -> Dict[int, WrapperAddress]:
        raise NotImplementedError()

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        try:
            if count <= 0:
                return []

            rospy.loginfo(f'address: {address}, count: {count}')
            register_entry = self._addressing[address]
            if register_entry.is_readable():
                rospy.loginfo(register_entry.read())
                return register_entry.read() + self.getValues(address + register_entry.size,
                                                              count - register_entry.size)

            return super().getValues(address, count)
        except Exception as e:
            rospy.logerr(e)
            traceback.print_exc()

    def setValues(self, address: int, values: List[bool]) -> None:
        register_entry = self._addressing[address]
        if register_entry.is_writable():
            return register_entry.write(values)

        super().setValues(address, values)
