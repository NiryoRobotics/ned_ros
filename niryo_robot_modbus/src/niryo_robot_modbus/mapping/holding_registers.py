import math

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper

from . import slave_context
from .abc_register_entries import (ABCRegisterEntries,
                                   ABCRegisterEntry,
                                   ABCConveyorRegisterEntries,
                                   ABCCommonStoreEntry,
                                   ABCUserStoreEntries,
                                   ABCStringEntries)
from ..CommonStore import CommonStore
from ..util import MoveType, CHAR_PER_BLOCK, MAX_STRING_LENGTH


@slave_context.holding_register
class AnalogOutputEntries(ABCRegisterEntries):
    data_type = float
    reserved_addresses = 50

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        analog_outputs = [ao.name for ao in ros_wrapper.get_analog_io_state().analog_outputs]
        return len(analog_outputs)

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        self._analog_outputs = [ao.name for ao in ros_wrapper.get_analog_io_state().analog_outputs]

    def get(self) -> float:
        pin_value = self._ros_wrapper.analog_read(self._analog_outputs[self._index])
        return pin_value

    def set(self, value: float):
        self._ros_wrapper.analog_write(self._analog_outputs[self._index], value)


@slave_context.holding_register
class JointTargetEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_joints())

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        CommonStore.joint_target = [0] * len(ros_wrapper.get_joints())

    def get(self) -> float:
        return CommonStore.joint_target[self._index]

    def set(self, value: float):
        CommonStore.joint_target[self._index] = value


@slave_context.holding_register
class PoseTargetEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_pose_as_list())

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        CommonStore.pose_target = [0] * len(ros_wrapper.get_pose_as_list())

    def get(self) -> float:
        return CommonStore.pose_target[self._index]

    def set(self, value: float):
        CommonStore.pose_target[self._index] = value


@slave_context.holding_register
class MoveTypeEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return CommonStore.move_type.value

    def set(self, value: int):
        CommonStore.move_type = MoveType(value)


@slave_context.holding_register
class TCPTransformationEntry(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_tcp(as_list=True))

    def get(self) -> float:
        current_tcp = self._ros_wrapper.get_tcp(as_list=True)
        return current_tcp[self._index]

    def set(self, value: float) -> None:
        current_tcp = self._ros_wrapper.get_tcp(as_list=True)
        current_tcp[self._index] = value
        self._ros_wrapper.set_tcp(*current_tcp)


@slave_context.holding_register
class ConveyorSpeedEntries(ABCConveyorRegisterEntries):
    data_type = int

    def get(self) -> int:
        feedback = self._safe_conveyor_feedback()
        return feedback.speed

    def set(self, value: int) -> None:
        self._safe_control_conveyor(speed=value, bool_control_on=True)


@slave_context.holding_register
class GripperOpenSpeedEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'gripper_open_speed'


@slave_context.holding_register
class GripperOpenMaxTorqueEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'gripper_open_max_torque'


@slave_context.holding_register
class GripperOpenHoldTorqueEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'gripper_open_hold_torque'


@slave_context.holding_register
class GripperCloseSpeedEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'gripper_close_speed'


@slave_context.holding_register
class GripperCloseMaxTorqueEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'gripper_close_max_torque'


@slave_context.holding_register
class GripperCloseHoldTorqueEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'gripper_close_hold_torque'


@slave_context.holding_register
class RelativeXEntry(ABCCommonStoreEntry):
    data_type = float
    common_store_attribute = 'relative_x'


@slave_context.holding_register
class RelativeYEntry(ABCCommonStoreEntry):
    data_type = float
    common_store_attribute = 'relative_y'


@slave_context.holding_register
class RelativeYawEntry(ABCCommonStoreEntry):
    data_type = float
    common_store_attribute = 'relative_yaw'


@slave_context.holding_register
class WorkspaceNameEntries(ABCStringEntries):

    data_type = str

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return math.ceil(MAX_STRING_LENGTH / CHAR_PER_BLOCK)

    def get(self) -> str:
        try:
            return CommonStore.workspace_name[self._index:self._upper_index]
        except IndexError:
            return ''

    def set(self, value: str):
        while len(CommonStore.workspace_name) < self._index:
            CommonStore.workspace_name += ' '
        workspace_name_prefix = CommonStore.workspace_name[:self._index]
        workspace_name_suffix = CommonStore.workspace_name[self._upper_index:]
        CommonStore.workspace_name = workspace_name_prefix + value + workspace_name_suffix


@slave_context.holding_register
class HeightOffsetEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'height_offset'


@slave_context.holding_register
class TargetShapeEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'target_shape'


@slave_context.holding_register
class TargetColorEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'target_color'


@slave_context.holding_register
class ArmSpeedEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._ros_wrapper.get_max_velocity_scaling_factor().data

    def set(self, value: int) -> None:
        self._ros_wrapper.set_arm_max_velocity(value)


@slave_context.holding_register
class FloatUserStoreEntries(ABCUserStoreEntries):
    data_type = float
    starting_address = 200
