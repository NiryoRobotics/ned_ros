from typing import Dict, List

from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from pymodbus.datastore import ModbusBaseSlaveContext

from . import logger
from .abc_register_entries import ABCRegisterEntry, ABCRegisterEntries
from .CommonStore import CommonStore
from .util import RegisterType, PayloadHandler


class CustomModbusSlaveContext(ModbusBaseSlaveContext):

    def __init__(self):
        self.__payload_handler = PayloadHandler()
        self.__registered_entries = {
            RegisterType.COIL: [],
            RegisterType.INPUT_REGISTER: [],
            RegisterType.HOLDING_REGISTER: [],
            RegisterType.DISCRETE_INPUT: [],
        }

        self.__mapping: Dict[int, ABCRegisterEntry] = {}

    # - Mandatory ModbusBaseSlaveContext methods - #

    def reset(self):
        pass

    def validate(self, fx, address, count=1):
        register_type = RegisterType.from_function_code(fx)
        addresses = range(address, address + count)
        return all(self.exists(register_type, offset) for offset in addresses)

    def getValues(self, fx, address, count=1):
        try:
            logger.info(f'getValues: fx: {fx}, address: {address}, count: {count}')
            values = []
            for offset in range(0, count):
                entry = self.__mapping[self.__to_index(fx, address)]
                values += self.__payload_handler.encode(entry.read(), entry.data_type)
            return values
        except Exception as e:
            logger.exception(f'getValues: {e}')

    def setValues(self, fx, address, values):
        try:
            entry = self.__mapping[self.__to_index(fx, address)]
            decoded_payload = self.__payload_handler.decode(values, entry.data_type)
            entry.write(decoded_payload)
            logger.info(f'setValues: fx: {fx}, address: {address}, values: {values}')
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
        function_code_bytes = function_code.to_bytes(1, 'big')
        address_bytes = address.to_bytes(2, 'big')
        return int.from_bytes(function_code_bytes + address_bytes, byteorder='big')

    def exists(self, register_type: RegisterType, address: int) -> bool:
        """
        Check if the given address exists in the given register type
        :param register_type: a modbus register type
        :param address: a register address
        :return: True if the given address exists in the given register
        """
        return self.__to_index(register_type.value[0], address) in self.__mapping

    def write(self, function_code: int, address: int, payload: List[int]) -> None:
        entry = self.__mapping[self.__to_index(function_code, address)]
        decoded_payload = self.__payload_handler.decode(payload, entry.data_type)
        entry.write(decoded_payload)

    def add(self, register_type: RegisterType, register_entry: ABCRegisterEntry) -> None:
        if register_type.is_read_only() and not register_entry.is_read_only():
            logger.warning(f'Entry" {register_entry.__class__.__name__}" is not read-only but'
                           f'has been assigned to "{register_type.name}" which is read-only')
        if not register_type.is_read_only() and register_entry.is_read_only():
            raise ValueError(f'Entry "{register_entry.__class__.__name__}" is read-only but has been assigned to'
                             f'({register_type.name}) which is not read-only')
        self.__mapping[self.__to_index(register_type.value[0], register_entry.address)] = register_entry
        if not register_type.is_read_only():
            self.__mapping[self.__to_index(register_type.value[1], register_entry.address)] = register_entry

    def build_register(self):
        ros_wrapper = NiryoRosWrapper()
        for register_type, entries in self.__registered_entries.items():
            for entry in entries:
                if issubclass(entry, ABCRegisterEntries):
                    n_addresses = entry.get_address_count(ros_wrapper)
                    for offset in range(0, n_addresses):
                        inst = entry(ros_wrapper, offset)
                        self.add(register_type, inst)
                elif issubclass(entry, ABCRegisterEntry):
                    inst = entry(ros_wrapper)
                    self.add(register_type, inst)
                else:
                    raise TypeError(f'{entry.__name__} is not an ABCRegisterEntry subclass')

    # - class decorators - #

    def coil(self, address: int):

        def decorator(cls: ABCRegisterEntry):
            cls.starting_address = address
            cls.data_type = bool
            self.__registered_entries[RegisterType.COIL].append(cls)
            return cls

        return decorator
