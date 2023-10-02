from abc import ABC, abstractmethod
from typing import List

from pymodbus.datastore import ModbusSparseDataBlock

from .addressing.WrapperAddress import WrapperAddress


class WrapperDataBlock(ABC, ModbusSparseDataBlock):

    def __init__(self, ros_wrapper):
        self._ros_wrapper = ros_wrapper
        self._addressing = self._get_addressing()
        super().__init__({a: False for a in self._addressing.keys()})

    @abstractmethod
    def _get_addressing(self):
        raise NotImplementedError()

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        if count <= 0:
            return []

        register_entry = self._addressing[address]
        if isinstance(register_entry, WrapperAddress):
            return [register_entry.read()] + self.getValues(address + 1, count - 1)

        return super().getValues(address, count)

    def setValues(self, address: int, values: List[bool]) -> None:
        register_entry = self._addressing[address]
        if isinstance(register_entry, WrapperAddress):
            return register_entry.write(values)

        super().setValues(address, values)
