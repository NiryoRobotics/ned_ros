from niryo_robot_python_ros_wrapper import PinState, NiryoRosWrapper

from . import slave_context
from .abc_register_entries import ABCRegisterEntry, ABCRegisterEntries, ABCVisionRegisterEntry, ABCCommonStoreEntry


@slave_context.discrete_input
class DigitalInputEntries(ABCRegisterEntries):
    reserved_addresses = 50

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        digital_inputs = ros_wrapper.get_digital_io_state().digital_inputs
        return len(digital_inputs)

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        self._dios = [di.name for di in ros_wrapper.get_digital_io_state().digital_inputs]

    def get(self) -> bool:
        return self._ros_wrapper.digital_read(self._dios[self._index]) == PinState.HIGH


@slave_context.discrete_input
class MotorConnectionEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.get_hardware_status().connection_up


@slave_context.discrete_input
class ExecutingCommandEntry(ABCCommonStoreEntry):
    common_store_attribute = 'is_executing_command'


@slave_context.discrete_input
class VisionTargetFoundEntry(ABCVisionRegisterEntry):

    def get(self) -> bool:
        return self._is_target_found()
