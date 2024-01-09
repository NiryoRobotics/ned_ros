from __future__ import annotations
from typing import List

from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState, ConveyorDirection, ConveyorID, ObjectShape, ObjectColor

from tools_interface.msg import Tool
from niryo_robot_status.msg import RobotStatus
from niryo_robot_tools_commander.api.tools_ros_wrapper_enums import ToolID

from .abc_register_entries import ABCRegisterEntry, ABCRegisterEntries, ABCConveyorRegisterEntries
from .CommonStore import CommonStore
from .CustomModbusSlaveContext import CustomModbusSlaveContext
from .util import MoveType

slave_context = CustomModbusSlaveContext()


@slave_context.coil(0)
class DigitalOutputEntries(ABCRegisterEntries):

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

    def write(self, value: bool):
        pin_state = PinState.HIGH if value else PinState.LOW
        self._ros_wrapper.digital_write(self._digital_outputs[self._index], pin_state)


@slave_context.coil(50)
class ToolIDEntry(ABCRegisterEntry):

    def read(self) -> bool:
        return self._ros_wrapper.get_current_tool_id() != ToolID.NONE

    def write(self, value: List[bool]):
        self._ros_wrapper.update_tool()


@slave_context.coil(51)
class ToolActuationEntry(ABCRegisterEntry):

    def read(self) -> bool:
        current_tool_state = self._ros_wrapper.get_current_tool_state()
        return current_tool_state in [Tool.GRIPPER_STATE_CLOSE, Tool.VACUUM_PUMP_STATE_PULLED]

    def write(self, value: bool) -> None:
        if value:
            self._ros_wrapper.grasp_with_tool()
        else:
            self._ros_wrapper.release_with_tool()


@slave_context.coil(52)
class TCPEnabledEntry(ABCRegisterEntry):

    def read(self) -> bool:
        return self._ros_wrapper.get_tcp().enabled

    def write(self, value: bool) -> None:
        self._ros_wrapper.enable_tcp(value)


@slave_context.coil(60)
class ConveyorAttachedEntries(ABCConveyorRegisterEntries):

    def read(self) -> bool:
        return self._safe_conveyor_feedback().connection_state

    def write(self, value: bool) -> None:
        result = ConveyorID.NONE
        while result.value != ConveyorID.NONE.value:
            result = self._ros_wrapper.set_conveyor()


@slave_context.coil(64)
class ConveyorRunningEntries(ABCConveyorRegisterEntries):

    def read(self) -> bool:
        return self._safe_conveyor_feedback().running

    def write(self, value: bool) -> None:
        self._safe_control_conveyor(bool_control_on=value)


@slave_context.coil(68)
class ConveyorDirectionEntries(ABCConveyorRegisterEntries):

    def read(self) -> bool:
        feedback = self._safe_conveyor_feedback()
        return feedback.direction == ConveyorDirection.FORWARD

    def write(self, value: bool) -> None:
        direction = ConveyorDirection.FORWARD if value else ConveyorDirection.BACKWARD
        self._safe_control_conveyor(direction=direction)


@slave_context.coil(80)
class RobotMovingEntry(ABCRegisterEntry):

    def read(self) -> bool:
        robot_status = self._ros_wrapper.get_robot_status().robot_status
        return robot_status == RobotStatus.MOVING

    def write(self, value: bool) -> None:
        if value:
            if CommonStore.move_type == MoveType.MOVE_LINEAR:
                self._ros_wrapper.move_linear_pose(*CommonStore.pose_target)
            elif CommonStore.move_type == MoveType.MOVE_POSE:
                self._ros_wrapper.move_pose(*CommonStore.pose_target)
            elif CommonStore.move_type == MoveType.MOVE_JOINT:
                self._ros_wrapper.move_joints(*CommonStore.joint_target)
        else:
            self._ros_wrapper.stop_move()


@slave_context.coil(81)
class LearningModeEntry(ABCRegisterEntry):

    def read(self) -> bool:
        return self._ros_wrapper.get_learning_mode()

    def write(self, value: bool) -> None:
        self._ros_wrapper.set_learning_mode(value)


@slave_context.coil(82)
class CalibrationNeededEntry(ABCRegisterEntry):

    def read(self) -> bool:
        return self._ros_wrapper.get_hardware_status().calibration_needed

    def write(self, value: bool) -> None:
        if value:
            self._ros_wrapper.request_new_calibration()


@slave_context.coil(83)
class CalibrationEntry(ABCRegisterEntry):

    def read(self) -> bool:
        return self._ros_wrapper.get_hardware_status().calibration_in_progress

    def write(self, value: bool) -> None:
        if value:
            self._ros_wrapper.calibrate_auto()


@slave_context.coil(200)
class UserStoreEntries(ABCRegisterEntries):

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return 100

    def read(self) -> bool:
        return CommonStore.coil_user_store[self._index]

    def write(self, value: bool) -> None:
        CommonStore.coil_user_store[self._index] = value
