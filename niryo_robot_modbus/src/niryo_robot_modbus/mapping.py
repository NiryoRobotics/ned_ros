from typing import List

from niryo_robot_python_ros_wrapper import PinState, NiryoRosWrapper

from .CustomModbusSlaveContext import CustomModbusSlaveContext
from .register_entries import ABCRegisterEntry, ABCRegisterEntries

slave_context = CustomModbusSlaveContext()


@slave_context.coil
class DigitalOutput(ABCRegisterEntries):

    @staticmethod
    def get_starting_address() -> int:
        return 0

    @staticmethod
    def get_data_type() -> type:
        return bool

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        digital_outputs = [do.name for do in ros_wrapper.get_digital_io_state().digital_outputs]
        return len(digital_outputs)

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        self._digital_outputs = [do.name for do in self._ros_wrapper.get_digital_io_state().digital_outputs]

    def read(self) -> bool:
        pin_state = self._ros_wrapper.digital_read(self._digital_outputs[self._index])
        return pin_state == PinState.HIGH

    def write(self, value: List[bool]):
        pin_state = PinState.HIGH if value[0] else PinState.LOW
        self._ros_wrapper.digital_write(self._digital_outputs[self._index], pin_state)
