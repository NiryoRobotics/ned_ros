import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_system_api_client import system_api_client

from .WrapperAddress import AnalogWrapperAddress


def get_addressing(ros_wrapper: NiryoRosWrapper):
    n_joints = len(ros_wrapper.get_joints())
    n_poses = len(ros_wrapper.get_pose_as_list())
    hw_version = int(ros_wrapper.get_hardware_version()[3]) if len(ros_wrapper.get_hardware_version()) >= 4 else 1
    n_can_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', []))
    n_ttl_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', []))
    n_conveyors = n_can_conveyors + n_ttl_conveyors
    n_analog_inputs = len(ros_wrapper.get_analog_io_state().analog_inputs)

    return {
        # movej
        **AnalogWrapperAddress.dynamic_addressing(
            0,
        )
    }
