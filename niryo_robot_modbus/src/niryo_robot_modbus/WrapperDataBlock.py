from typing import List, Dict, Iterable

from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.exceptions import ModbusIOException

import rospy
from .addressing.WrapperAddress import WrapperAddress


class WrapperDataBlock(ModbusSparseDataBlock):

    def __init__(self, addressing: Dict[int, WrapperAddress], real_data_block_range: Iterable = []):
        self.__addressing = addressing
        self.__real_data_block_range = list(real_data_block_range)
        self.__check_addresses_sanity()
        sparse_data_addresses = list(self.__addressing.keys()) + self.__real_data_block_range
        super().__init__({a: False for a in sparse_data_addresses})

    def __check_addresses_sanity(self):
        common_addresses = set.intersection(set(self.__addressing.keys()), set(self.__real_data_block_range))
        if len(common_addresses) != 0:
            raise ValueError('Addressing and real datas have common addresses')

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        try:
            if count <= 0:
                return []
            if address in self.__real_data_block_range:
                return super().getValues(address, count)
            try:
                register_result = self.__addressing[address].read()
            except (IndexError, KeyError) as error:
                raise ModbusIOException(str(error))

            return [register_result] + self.getValues(address + 1, count - 1)
        except Exception as e:
            rospy.logerr(e)

    def setValues(self, address: int, values: List[bool]) -> None:
        if address in self.__real_data_block_range:
            super().setValues(address, values)
        try:
            self.__addressing[address].write(values)
        except (IndexError, KeyError) as error:
            raise ModbusIOException(str(error))
