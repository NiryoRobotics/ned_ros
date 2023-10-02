import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_system_api_client import system_api_client

from .WrapperAddress import AnalogWrapperAddress
from conveyor_interface.msg import ConveyorFeedback
from niryo_robot_rpi.msg import AnalogIO


def safe_get(container, item, default_value):
    """
    Safely retrieves the value at the specified item from the given container.

    This function attempts to access the element at the specified ``item``
    within the provided ``container``. If the item is not found,
    it returns the specified ``default_value`` instead.

    :param container: The container to retrieve the value from (list or dict).
    :type container: any object implementing the __getitem__ method
    :param item: The item to access.
    :type item: Any
    :param default_value: The value to return if the item is not found.
    :type default_value: Any
    :return: The value at the specified item within the container, or ``default_value`` if the item is not found.
    :rtype: Any
    """
    try:
        return container[item]
    except (IndexError, KeyError):
        return default_value


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
        **AnalogWrapperAddress.dynamic_addressing(0, n_joints, lambda ix: ros_wrapper.get_joints()[ix], precision=3),
        # pose (xyz/rpy)
        **AnalogWrapperAddress.dynamic_addressing(10,
                                                  n_poses,
                                                  lambda ix: ros_wrapper.get_pose_as_list()[ix],
                                                  precision=3),
        403:  # raspberry temperature
        AnalogWrapperAddress(read=lambda: ros_wrapper.get_hardware_status().rpi_temperature),
        404:  # raspberry available disk size
        AnalogWrapperAddress(read=lambda: ros_wrapper.get_available_disk_size()),
        405:  # raspberry ros logs size
        AnalogWrapperAddress(read=lambda: ros_wrapper.get_ros_logs_size()),
        410:  # system version major number
        AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][0])),
        411:  # system version minor number
        AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][2])),
        412:  # system version patch number
        AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][4])),
        413:  # system version build number
        AnalogWrapperAddress(read=lambda: int(system_api_client.get_system_version_current().data['system'][7:9])),
        420:  # robot version
        AnalogWrapperAddress(read=lambda: hw_version),
        # conveyors speed
        **AnalogWrapperAddress.dynamic_addressing(500,
                                                  n_conveyors,
                                                  lambda ix: safe_get(
                                                      ros_wrapper.get_conveyors_feedback(),
                                                      ix,
                                                      ConveyorFeedback()
                                                  ).speed),
        # analog inputs state
        **AnalogWrapperAddress.dynamic_addressing(
            600,
            n_analog_inputs,
            lambda ix: safe_get(
                ros_wrapper.get_analog_io_state().analog_inputs,
                ix,
                AnalogIO()
            ).value,
            precision=3
        )
    }
