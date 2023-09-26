import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_system_api_client import system_api_client

from . import dynamic_addressing


def get_addressing(ros_wrapper: NiryoRosWrapper):
    n_joints = len(ros_wrapper.get_joints())
    n_poses = len(ros_wrapper.get_pose_as_list())
    hw_version = int(ros_wrapper.get_hardware_version()[3]) if len(ros_wrapper.get_hardware_version()) >= 4 else 1
    n_can_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', []))
    n_ttl_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', []))
    n_conveyors = n_can_conveyors + n_ttl_conveyors
    n_analog_inputs = len(ros_wrapper.get_analog_io_state().analog_inputs)

    return {
        # joints
        **dynamic_addressing(0, n_joints, lambda ix: ros_wrapper.get_joints()[ix]),
        # pose (xyz/rpy)
        **dynamic_addressing(10, n_poses, lambda ix: ros_wrapper.get_pose_as_list()[ix]),
        403:  # raspberry temperature
        lambda: ros_wrapper.get_hardware_status().rpi_temperature,
        404:  # raspberry available disk size
        lambda: ros_wrapper.get_available_disk_size(),
        405:  # raspberry ros logs size
        lambda: ros_wrapper.get_ros_logs_size(),
        410:  # system version major number
        lambda: system_api_client.get_system_version_current().data.system[0],
        411:  # system version minor number
        lambda: system_api_client.get_system_version_current().data.system[2],
        412:  # system version patch number
        lambda: system_api_client.get_system_version_current().data.system[6],
        413:  # system version build number
        lambda: system_api_client.get_system_version_current().data.system[7:9],
        420:  # robot version
        lambda: hw_version,
        # conveyors speed
        **dynamic_addressing(500, n_conveyors, lambda ix: ros_wrapper.get_conveyors_feedback()[ix].speed),
        # analog inputs state
        **dynamic_addressing(600, n_analog_inputs, lambda ix: ros_wrapper.get_analog_io_state().analog_inputs[ix].value)
    }  # yapf: disable
