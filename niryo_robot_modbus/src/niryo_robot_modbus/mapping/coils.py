from __future__ import annotations
from typing import List

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState, ConveyorDirection, ConveyorID, PinID

from tools_interface.msg import Tool

from niryo_robot_status.msg import RobotStatus
from niryo_robot_tools_commander.api.tools_ros_wrapper_enums import ToolID

from ..CommonStore import CommonStore
from ..util import MoveType
from . import slave_context
from .abc_register_entries import ABCRegisterEntry, ABCRegisterEntries, ABCConveyorRegisterEntries, ABCUserStoreEntries


@slave_context.coil
class DigitalOutputEntries(ABCRegisterEntries):
    reserved_addresses = 50

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        digital_outputs = ros_wrapper.get_digital_io_state().digital_outputs
        return len(digital_outputs)

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        self._digital_outputs = [do.name for do in self._ros_wrapper.get_digital_io_state().digital_outputs]

    def get(self) -> bool:
        pin_state = self._ros_wrapper.digital_read(self._digital_outputs[self._index])
        return pin_state == PinState.HIGH

    def set(self, value: bool):
        pin_state = PinState.HIGH if value else PinState.LOW
        self._ros_wrapper.digital_write(self._digital_outputs[self._index], pin_state)


@slave_context.coil
class ToolEquippedEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.get_current_tool_id() != ToolID.NONE

    def set(self, value: List[bool]):
        self._ros_wrapper.update_tool()


@slave_context.coil
class ToolActuationEntry(ABCRegisterEntry):

    def get(self) -> bool:
        current_tool_state = self._ros_wrapper.get_current_tool_state()
        return current_tool_state in [Tool.GRIPPER_STATE_CLOSE, Tool.VACUUM_PUMP_STATE_PULLED]

    def set(self, value: bool) -> None:
        tool_id = self._ros_wrapper.get_current_tool_id()

        if tool_id == ToolID.NONE:
            return

        action_is_grasp = value
        tool_is_gripper = tool_id in [ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4]

        if action_is_grasp and tool_is_gripper:
            self._ros_wrapper.close_gripper(CommonStore.gripper_close_speed,
                                            CommonStore.gripper_close_max_torque,
                                            CommonStore.gripper_close_hold_torque)
        elif action_is_grasp and not tool_is_gripper:
            self._ros_wrapper.grasp_with_tool(PinID.DO4)
        elif tool_is_gripper:
            self._ros_wrapper.open_gripper(CommonStore.gripper_open_speed,
                                           CommonStore.gripper_open_max_torque,
                                           CommonStore.gripper_open_hold_torque)
        else:
            self._ros_wrapper.release_with_tool(PinID.DO4)


@slave_context.coil
class TCPEnabledEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.get_tcp().enabled

    def set(self, value: bool) -> None:
        self._ros_wrapper.enable_tcp(value)


@slave_context.coil
class ConveyorAttachedEntries(ABCConveyorRegisterEntries):

    def get(self) -> bool:
        return self._safe_conveyor_feedback().conveyor_id != 0

    def set(self, value: bool) -> None:
        if value and self._conveyor_number == ConveyorID.NONE:
            self._ros_wrapper.set_conveyor()
        elif not value:
            self._ros_wrapper.unset_conveyor(self._conveyor_number)


@slave_context.coil
class ConveyorRunningEntries(ABCConveyorRegisterEntries):

    def get(self) -> bool:
        return self._safe_conveyor_feedback().running

    def set(self, value: bool) -> None:
        self._safe_control_conveyor(bool_control_on=value)


@slave_context.coil
class ConveyorDirectionEntries(ABCConveyorRegisterEntries):

    def get(self) -> bool:
        feedback = self._safe_conveyor_feedback()
        direction = feedback.direction == ConveyorDirection.FORWARD
        return direction

    def set(self, value: bool) -> None:
        direction = ConveyorDirection.FORWARD if value else ConveyorDirection.BACKWARD
        self._safe_control_conveyor(direction=direction)


@slave_context.coil
class RobotMovingEntry(ABCRegisterEntry):

    def get(self) -> bool:
        robot_status = self._ros_wrapper.get_robot_status().robot_status
        return robot_status == RobotStatus.MOVING

    def set(self, value: bool) -> None:
        if value:
            if CommonStore.move_type == MoveType.MOVE_LINEAR:
                self._ros_wrapper.move_linear_pose(*CommonStore.pose_target)
            elif CommonStore.move_type == MoveType.MOVE_POSE:
                self._ros_wrapper.move_pose(*CommonStore.pose_target)
            elif CommonStore.move_type == MoveType.MOVE_JOINT:
                self._ros_wrapper.move_joints(*CommonStore.joint_target)
            else:
                raise ValueError(f"Unknown move type: {CommonStore.move_type}")
        else:
            self._ros_wrapper.stop_move()


@slave_context.coil
class LearningModeEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.get_learning_mode()

    def set(self, value: bool) -> None:
        self._ros_wrapper.set_learning_mode(value)


@slave_context.coil
class CalibrationNeededEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.get_hardware_status().calibration_needed

    def set(self, value: bool) -> None:
        if value:
            self._ros_wrapper.request_new_calibration()


@slave_context.coil
class CalibrationEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.get_hardware_status().calibration_in_progress

    def set(self, value: bool) -> None:
        if value:
            self._ros_wrapper.calibrate_auto()


@slave_context.coil
class CollisionDetectedEntry(ABCRegisterEntry):

    def get(self) -> bool:
        return self._ros_wrapper.collision_detected

    def set(self, value: bool):
        if value is False:
            self._ros_wrapper.clear_collision_detected()


@slave_context.coil
class CoilUserStoreEntries(ABCUserStoreEntries):
    starting_address = 200
