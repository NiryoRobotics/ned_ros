from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from niryo_robot_system_api_client import system_api_client

from . import slave_context
from .abc_register_entries import (
    ABCRegisterEntry,
    ABCRegisterEntries,
    ABCVisionRegisterEntry,
    ABCConveyorRegisterEntries,
    ABCCommonStoreEntry,
)
from ..CommonStore import CommonStore


@slave_context.input_register
class AnalogInputEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        analog_inputs = ros_wrapper.get_analog_io_state().analog_inputs
        return len(analog_inputs)

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper, ix)
        self._analog_inputs = [ai.name for ai in ros_wrapper.get_analog_io_state().analog_inputs]

    def get(self) -> float:
        pin_value = self._ros_wrapper.analog_read(self._analog_inputs[self._index])
        return pin_value


@slave_context.input_register
class CurrentJointStateEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_joints())

    def get(self) -> float:
        return self._ros_wrapper.get_joints()[self._index]


@slave_context.input_register
class CurrentPoseStateEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_pose_as_list())

    def get(self) -> float:
        return self._ros_wrapper.get_pose_as_list()[self._index]


@slave_context.input_register
class AbsoluteFromRelativePoseEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_pose_as_list())

    def get(self) -> float:
        target_pose = self._ros_wrapper.get_target_pose_from_rel(CommonStore.workspace_name,
                                                                 CommonStore.height_offset,
                                                                 CommonStore.relative_x,
                                                                 CommonStore.relative_y,
                                                                 CommonStore.relative_yaw)
        return [
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            target_pose.rpy.roll,
            target_pose.rpy.pitch,
            target_pose.rpy.yaw
        ][self._index]


@slave_context.input_register
class VisionTargetPoseEntries(ABCRegisterEntries):
    data_type = float

    @staticmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        return len(ros_wrapper.get_pose_as_list())

    def get(self) -> float:
        return self._ros_wrapper.get_target_pose_from_cam(CommonStore.workspace_name,
                                                          CommonStore.height_offset,
                                                          CommonStore.target_shape,
                                                          CommonStore.target_color)[1][self._index]


@slave_context.input_register
class VisionTargetShapeEntry(ABCVisionRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._get_vision_target()['shape']


@slave_context.input_register
class VisionTargetColorEntry(ABCVisionRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._get_vision_target()['color']


@slave_context.input_register
class ConveyorIDEntries(ABCConveyorRegisterEntries):
    data_type = int

    def get(self) -> int:
        return self._safe_conveyor_feedback().conveyor_id


@slave_context.input_register
class LastCommandResultEntry(ABCCommonStoreEntry):
    data_type = int
    common_store_attribute = 'last_command_result'


@slave_context.input_register
class CurrentToolIDEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._ros_wrapper.get_current_tool_id()


@slave_context.input_register
class RaspberryTemperatureEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._ros_wrapper.get_hardware_status().rpi_temperature


@slave_context.input_register
class RaspberryAvailableDiskSizeEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._ros_wrapper.get_available_disk_size()


@slave_context.input_register
class RaspberryLogsSizeEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return self._ros_wrapper.get_ros_logs_size()


@slave_context.input_register
class HardwareVersionEntry(ABCRegisterEntry):
    data_type = str

    def get(self) -> str:
        return self._ros_wrapper.get_hardware_version()


@slave_context.input_register
class SystemVersionMajorEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return int(system_api_client.get_system_version_current().data['system'][0])


@slave_context.input_register
class SystemVersionMinorEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return int(system_api_client.get_system_version_current().data['system'][2])


@slave_context.input_register
class SystemVersionPatchEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return int(system_api_client.get_system_version_current().data['system'][4])


@slave_context.input_register
class SystemVersionBuildEntry(ABCRegisterEntry):
    data_type = int

    def get(self) -> int:
        return int(system_api_client.get_system_version_current().data['system'][7:9])
