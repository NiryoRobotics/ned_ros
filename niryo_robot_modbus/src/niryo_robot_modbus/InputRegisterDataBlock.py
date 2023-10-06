import rospy

from niryo_robot_system_api_client import system_api_client

from conveyor_interface.msg import ConveyorFeedback
from niryo_robot_rpi.msg import AnalogIO

from . import safe_get
from .WrapperDataBlock import WrapperDataBlock
from .WrapperAddress import AnalogWrapperAddress
"""
Input register:
xx	Last command result
xx  conveyor 1 id
xx  conveyor 2 id
xx*6 vision target pose
xx vision - Shape of the object found (-1: ANY, 1: CIRCLE, 2: SQUARE, 3: TRIANGLE, 0: NONE)
xx vision - Color of the object found (-1: ANY, 1: BLUE, 2: RED, 3: GREEN, 0: NONE)
xx tool id
xx*6 pose from pose relative to workspace (using relative x, relative y, relative yaw and height offset)
"""


class InputRegisterDataBlock(WrapperDataBlock):

    def _get_addressing(self):
        n_joints = len(self._ros_wrapper.get_joints())
        n_poses = len(self._ros_wrapper.get_pose_as_list())
        hw_version = int(self._ros_wrapper.get_hardware_version()[3]) if len(
            self._ros_wrapper.get_hardware_version()) >= 4 else 1
        n_can_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', []))
        n_ttl_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', []))
        n_conveyors = n_can_conveyors + n_ttl_conveyors
        n_analog_inputs = len(self._ros_wrapper.get_analog_io_state().analog_inputs)

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
            ** {
                a: AnalogWrapperAddress(None, None, data_type=int) for a in range(200, 300)
            },
            403:  # raspberry temperature
            AnalogWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().rpi_temperature, data_type=int),
            404:  # raspberry available disk size
            AnalogWrapperAddress(read=lambda: self._ros_wrapper.get_available_disk_size(), data_type=int),
            405:  # raspberry ros logs size
            AnalogWrapperAddress(read=lambda: self._ros_wrapper.get_ros_logs_size(), data_type=int),
            410:  # system version major number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][0]),
                                 data_type=int),
            411:  # system version minor number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][2]),
                                 data_type=int),
            412:  # system version patch number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][4]),
                                 data_type=int),
            413:  # system version build number
            AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][7:9]),
                                 data_type=int),
            420:  # robot version
            AnalogWrapperAddress(read=lambda: hw_version, data_type=int),
            # conveyors speed
            **AnalogWrapperAddress.dynamic_addressing(
                500,
                n_conveyors,
                lambda ix: safe_get(self._ros_wrapper.get_conveyors_feedback(), ix, ConveyorFeedback()).speed,
                data_type=float,
            ),
            # analog inputs state
            **AnalogWrapperAddress.dynamic_addressing(
                600,
                n_analog_inputs,
                lambda ix: safe_get(self._ros_wrapper.get_analog_io_state().analog_inputs, ix, AnalogIO()).value,
                data_type=float,
            )
        }
