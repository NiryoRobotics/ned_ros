from niryo_robot_system_api_client import system_api_client
from .conversions import object_color_to_int, object_shape_to_int
from .WrapperDataBlock import WrapperDataBlock
from .WrapperAddress import AnalogWrapperAddress


class InputRegisterDataBlock(WrapperDataBlock):

    def _get_addressing(self):
        n_joints = len(self._ros_wrapper.get_joints())
        n_poses = len(self._ros_wrapper.get_pose_as_list())
        full_hw_version = self._ros_wrapper.get_hardware_version()
        hw_version = int(full_hw_version[3]) if len(full_hw_version) >= 4 else 1
        n_analog_inputs = len(self._ros_wrapper.get_analog_io_state().analog_inputs)
        n_conveyors = 2

        return {
            # joints
            **AnalogWrapperAddress.dynamic_addressing(0,
                                                      n_joints,
                                                      lambda ix: self._ros_wrapper.get_joints()[ix],
                                                      data_type=float),
            # pose (xyz/rpy)
            **AnalogWrapperAddress.dynamic_addressing(12,
                                                      n_poses,
                                                      lambda ix: self._ros_wrapper.get_pose_as_list()[ix],
                                                      data_type=float),
            # absolute pose of the relative pose defined in the holding register
            **AnalogWrapperAddress.dynamic_addressing(24,
                                                      n_poses,
                                                      lambda ix: self.pose_from_shared_relative()[ix],
                                                      data_type=float),
            # vision target pose
            **AnalogWrapperAddress.dynamic_addressing(
                36,
                n_poses,
                read=lambda ix: self._ros_wrapper.get_target_pose_from_rel(self._shared_data.workspace_name,
                                                                           self._shared_data.height_offset,
                                                                           self._shared_data.shape,
                                                                           self._shared_data.color)[1][ix],
                data_type=float
            ),
            # vision target shape
            50: AnalogWrapperAddress(read=lambda: object_shape_to_int[self._ros_wrapper.detect_object(
                self._shared_data.workspace_name,
                self._shared_data.shape,
                self._shared_data.color)
            ]),
            # vision target color
            51: AnalogWrapperAddress(read=lambda: object_color_to_int[self._ros_wrapper.detect_object(
                self._shared_data.workspace_name,
                self._shared_data.shape,
                self._shared_data.color)
            ]),
            55:  # last command result
            AnalogWrapperAddress(read=lambda: self._shared_data.is_executing_command),
            56:  # current tool id
            AnalogWrapperAddress(read=self._ros_wrapper.get_current_tool_id),
            # conveyor ids
            **AnalogWrapperAddress.dynamic_addressing(60,
                                                      n_conveyors,
                                                      read=lambda ix: self._safe_conveyor_feedback(ix).conveyor_id),
            70:  # raspberry temperature
            AnalogWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().rpi_temperature, data_type=int),
            71:  # raspberry available disk size
            AnalogWrapperAddress(read=self._ros_wrapper.get_available_disk_size, data_type=int),
            72:  # raspberry ros logs size
            AnalogWrapperAddress(read=self._ros_wrapper.get_ros_logs_size, data_type=int),
            73:  # robot version
            AnalogWrapperAddress(read=lambda: hw_version, data_type=int),
            74:  # system version major number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][0]),
                                 data_type=int),
            75:  # system version minor number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][2]),
                                 data_type=int),
            76:  # system version patch number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][4]),
                                 data_type=int),
            77:  # system version build number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][7:9]),
                                 data_type=int),
            # analog inputs state
            **AnalogWrapperAddress.dynamic_addressing(
                100,
                n_analog_inputs,
                lambda ix: self._ros_wrapper.get_analog_io_state().analog_inputs[ix].value,
                data_type=float,
            )
        }

    def pose_from_shared_relative(self):
        target_pose = self._ros_wrapper.get_target_pose_from_rel(self._shared_data.workspace_name,
                                                                 self._shared_data.height_offset,
                                                                 self._shared_data.relative_x,
                                                                 self._shared_data.relative_y,
                                                                 self._shared_data.relative_yaw)
        return [
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            target_pose.rpy.roll,
            target_pose.rpy.pitch,
            target_pose.rpy.yaw
        ]
