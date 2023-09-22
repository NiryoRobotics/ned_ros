from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_system_api_client import system_api_client

n = NiryoRosWrapper()
"""
TODO: pass these values to discrete input
300: Learning Mode activated
400: Motors connection up (Ok = 1, Not ok = 0)
401: Calibration needed flag
402: Calibration in progress flag
530: Conveyor 1 connection state (Connected = 1 , Not connected = 0)
531: Conveyor 1 control status ( On = 0, Off = 1)
533	Conveyor 1 direction (Backward = -1, Forward = 1)
540: Conveyor 2 connection state (Connected = 1 , Not connected = 0)
541: Conveyor 2 control status ( On = 0, Off = 1)
543	Conveyor 2 direction (Backward = -1, Forward = 1)
600 - 604: Analog IO mode
"""


def get_mapping(ros_wrapper: NiryoRosWrapper):
    return {
        0: lambda: ros_wrapper.get_joints()[0],
        1: lambda: ros_wrapper.get_joints()[1],
        2: lambda: ros_wrapper.get_joints()[2],
        3: lambda: ros_wrapper.get_joints()[3],
        4: lambda: ros_wrapper.get_joints()[4],
        5: lambda: ros_wrapper.get_joints()[5],
        6: lambda: ros_wrapper.get_joints()[6],
        10: lambda: ros_wrapper.get_pose().position.x,
        11: lambda: ros_wrapper.get_pose().position.y,
        12: lambda: ros_wrapper.get_pose().position.z,
        13: lambda: ros_wrapper.get_pose().rpy.roll,
        14: lambda: ros_wrapper.get_pose().rpy.pitch,
        15: lambda: ros_wrapper.get_pose().rpy.yaw,
        403: lambda: ros_wrapper.get_hardware_status().rpi_temperature,
        404: lambda: ros_wrapper.get_available_disk_size(),
        405: lambda: ros_wrapper.get_ros_logs_size(),
        410: lambda: system_api_client.get_system_version_current().data.system[0],
        411: lambda: system_api_client.get_system_version_current().data.system[2],
        412: lambda: system_api_client.get_system_version_current().data.system[6],
        413: lambda: system_api_client.get_system_version_current().data.system[7:9],
        420: lambda: int(ros_wrapper.get_hardware_version()[3]) if len(ros_wrapper.get_hardware_version()) >= 4 else 1,
        532: lambda: ros_wrapper.get_conveyors_feedback()[0].speed,
        542: lambda: ros_wrapper.get_conveyors_feedback()[1].speed,
        610: lambda: ros_wrapper.get_analog_io_state().analog_inputs[0].value,
        611: lambda: ros_wrapper.get_analog_io_state().analog_inputs[1].value,
    }
