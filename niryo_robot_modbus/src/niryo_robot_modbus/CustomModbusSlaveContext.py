from typing import Dict, List

from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from pymodbus.datastore import ModbusBaseSlaveContext
from pymodbus.payload import BinaryPayloadDecoder

from . import logger
from .mapping.abc_register_entries import ABCRegisterEntry, ABCRegisterEntries
from .util import RegisterType, PayloadHandler, get_data_type_addresses_number


class CustomModbusSlaveContext(ModbusBaseSlaveContext):

    def __init__(self):
        self.__payload_handler = PayloadHandler()
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
            addresses = range(address, address + count)
            return all(self.__to_index(fx, offset) in self.__mapping for offset in addresses)
        except Exception as e:
            logger.exception(e)

    def getValues(self, fx, address, count=1):
        try:
            if count == 0:
                return []

            logger.info(f'getValues: fx: {fx}, address: {address}, count: {count}')
            entry = self.__mapping[self.__to_index(fx, address)]
            n_address_per_entry = get_data_type_addresses_number(entry.data_type)
            return entry.read() + self.getValues(fx, address + n_address_per_entry, count - 1)
        except Exception as e:
            logger.exception(f'getValues: {e}')

    def setValues(self, fx, address, values):
        try:
            if not values:
                return

            # logger.info(f'setValues: fx: {fx}, address: {address}, values: {values}')
            # convert any multiple write code to single write as we only use single writes internally
            fx %= 10
            entry = self.__mapping[self.__to_index(fx, address)]
            n_address_per_entry = get_data_type_addresses_number(entry.data_type)
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

    def exists(self, register_type: RegisterType, address: int) -> bool:
        """
        Check if the given address exists in the given register type
        :param register_type: a modbus register type
        :param address: a register address
        :return: True if the given address exists in the given register
        """
        return self.__to_index(register_type.value[0], address) in self.__mapping

    def build_register(self):
        ros_wrapper = NiryoRosWrapper()

        for register_type, entries in self.__registered_entries.items():
            register_offset = 0
            for entry in entries:
                address = register_offset
                n_address_per_entry = get_data_type_addresses_number(entry.data_type)
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

    def print_registers(self):
        for index, entry in self.__mapping.items():
            text = f'{self.__from_index(index)}({index}): {entry.__class__.__name__}'
            logger.info(text)

    def pretty_print_registers(self):
        reversed_mapping = {r: {} for r in RegisterType}
        for index, entry in self.__mapping.items():
            function_code, address = self.__from_index(index)
            register_type = RegisterType.from_function_code(function_code)
            if entry.__class__ not in reversed_mapping[register_type]:
                reversed_mapping[register_type][entry.__class__] = set()
            reversed_mapping[register_type][entry.__class__].add(address)

        for register, entries in reversed_mapping.items():
            for entry, addresses in entries.items():
                addresses = sorted(addresses)
                if len(addresses) < 2:
                    logger.info(f'| {addresses[0]} | {entry.__name__} |')
                else:
                    logger.info(f'| {addresses[0]} - {addresses[-1]} | {entry.__name__} |')

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
