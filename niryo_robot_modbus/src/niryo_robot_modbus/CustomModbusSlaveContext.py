from typing import Dict, List, Tuple

from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from pymodbus.datastore import ModbusBaseSlaveContext

from . import logger
from .mapping.abc_register_entries import ABCRegisterEntry, ABCRegisterEntries
from .util import RegisterType


class CustomModbusSlaveContext(ModbusBaseSlaveContext):

    def __init__(self):
        self.__registered_entries = {
            RegisterType.COIL: [],
            RegisterType.INPUT_REGISTER: [],
            RegisterType.HOLDING_REGISTER: [],
            RegisterType.DISCRETE_INPUT: [],
        }

        self.__mapping: Dict[int, ABCRegisterEntries] = {}

    # - Mandatory ModbusBaseSlaveContext methods - #

    def reset(self):
        pass

    def validate(self, fx, address, count=1):
        try:
            # convert any multiple write code to single write as we only use single writes internally
            fx %= 10
            logger.debug(f'validate: fx: {fx}, address: {address}, count: {count}')
            addresses = range(address, address + count)
            return all(self.__to_index(fx, offset) in self.__mapping for offset in addresses)
        except Exception as e:
            logger.exception(e)

    def getValues(self, fx, address, count=1):
        try:
            if count <= 0:
                return []

            logger.debug(f'getValues: fx: {fx}, address: {address}, count: {count}')
            entry = self.__mapping[self.__to_index(fx, address)]
            entry_read = entry.read()
            if len(entry_read) <= 0:
                return []
            elif len(entry_read) > count:
                entry_read = entry_read[:count]
            return entry_read + self.getValues(fx, address + len(entry_read), count - len(entry_read))
        except Exception as e:
            logger.exception(f'getValues: {e}')

    def setValues(self, fx, address, values):
        try:
            if not values:
                return

            logger.debug(f'setValues: fx: {fx}, address: {address}, values: {values}')
            # convert any multiple write code to single write as we only use single writes internally
            fx %= 10
            entry = self.__mapping[self.__to_index(fx, address)]
            n_address_per_entry = entry.get_n_addresses()
            entry.write(values[:n_address_per_entry])

            self.setValues(fx, address + n_address_per_entry, values[n_address_per_entry:])
        except Exception as e:
            logger.exception(e)

    # - Custom slave context functions - #

    @staticmethod
    def __to_index(function_code: int, address: int) -> int:
        """
        Generate a unique index based on the function code and the address
        :param function_code: a modbus register function code
        :param address: a register address
        :return: a unique index based on the function code and the address
        """
        assert address < 2**16, f'address must be lesser than {2**16}'
        # multiple_write_function_codes are equal to single_write_function_codes + 10
        function_code %= 10
        function_code_bytes = function_code.to_bytes(1, 'big')
        address_bytes = address.to_bytes(2, 'big')
        return int.from_bytes(function_code_bytes + address_bytes, 'big')

    @staticmethod
    def __from_index(index: int) -> (int, int):
        """
        Extract the function code and address from the unique index
        :param index: a unique index generated using __to_index
        :return: a tuple containing the function code and address
        """
        index_bytes = index.to_bytes(3, 'big')
        function_code = int.from_bytes(index_bytes[:1], 'big')
        address = int.from_bytes(index_bytes[1:], 'big')
        return function_code, address

    def build_register(self) -> None:
        """
        Must be called before starting the server
        Build the register according to each entry's specs by attributing an address to each one of them
        """
        ros_wrapper = NiryoRosWrapper()
        ros_wrapper.wait_for_node_initialization('niryo_robot_rpi')

        for register_type, entries in self.__registered_entries.items():
            register_offset = 0
            for entry in entries:
                if entry.starting_address > 0:
                    register_offset = entry.starting_address
                address = register_offset
                n_address_per_entry = entry.get_n_addresses()
                n_addresses = entry.get_address_count(ros_wrapper)
                for ix in range(n_addresses):
                    if issubclass(entry, ABCRegisterEntry):
                        inst = entry(ros_wrapper)
                    else:
                        inst = entry(ros_wrapper, ix)

                    for i in range(n_address_per_entry):
                        self.__mapping[self.__to_index(register_type.value[0], address + i)] = inst
                        if not register_type.is_read_only():
                            self.__mapping[self.__to_index(register_type.value[1], address + i)] = inst

                    address += n_address_per_entry

                register_offset += entry.reserved_addresses or n_addresses * n_address_per_entry

    def get_addressing(self) -> Dict[RegisterType, Dict[Tuple[int, int], ABCRegisterEntries]]:
        reversed_mapping = {r: {} for r in RegisterType}
        for index, entry in self.__mapping.items():
            function_code, address = self.__from_index(index)
            register_type = RegisterType.from_function_code(function_code)
            if entry.__class__ not in reversed_mapping[register_type]:
                reversed_mapping[register_type][entry.__class__] = set()
            reversed_mapping[register_type][entry.__class__].add(address)

        addressing = {}
        for register_type, entries in reversed_mapping.items():
            addressing[register_type] = {}
            for entry, addresses in entries.items():
                addresses = sorted(addresses)
                addressing[register_type][(addresses[0], addresses[-1])] = entry
        return addressing

    def pretty_print_registers(self):
        for register_type, entries in self.get_addressing().items():
            logger.debug('==============')
            logger.debug(register_type.name)
            logger.debug('==============')
            for (low_addr, high_addr), entry in entries.items():
                if low_addr == high_addr:
                    logger.debug(f'| {low_addr} | {entry.__name__} |')
                else:
                    logger.debug(f'| {low_addr} - {high_addr} | {entry.__name__} |')

    # - class decorators - #

    def __decorator(self, cls: ABCRegisterEntries, register_type: RegisterType, data_type: type):
        cls.data_type = data_type
        self.__registered_entries[register_type].append(cls)
        return cls

    def coil(self, cls: ABCRegisterEntries) -> ABCRegisterEntries:
        return self.__decorator(cls, RegisterType.COIL, bool)

    def discrete_input(self, cls: ABCRegisterEntries) -> ABCRegisterEntries:
        return self.__decorator(cls, RegisterType.DISCRETE_INPUT, bool)

    def holding_register(self, cls: ABCRegisterEntries):
        return self.__decorator(cls, RegisterType.HOLDING_REGISTER, cls.data_type)

    def input_register(self, cls: ABCRegisterEntries):
        return self.__decorator(cls, RegisterType.INPUT_REGISTER, cls.data_type)
