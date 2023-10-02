import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, PinState

from .WrapperAddress import DigitalWrapperAddress


def get_addressing(ros_wrapper: NiryoRosWrapper):
    """
    This function creates a dynamic addressing scheme for reading various hardware components' states,
    such as digital inputs, learning mode, calibration status, motor connection status and conveyor status.

    :param ros_wrapper: An instance of the NiryoRosWrapper class providing ROS interface.
    :type ros_wrapper: NiryoRosWrapper
    :returns: A dictionary mapping integer indices and keys to DigitalWrapperAddress objects.
    :rtype: Dict[int, DigitalWrapperAddress]
    """

    n_digital_inputs = len(ros_wrapper.get_digital_io_state().digital_inputs)
    n_can_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', []))
    n_ttl_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', []))
    n_conveyors = n_can_conveyors + n_ttl_conveyors
    dios = [di.name for di in ros_wrapper.get_digital_io_state().digital_inputs]

    return {
        # Digital inputs states
        **DigitalWrapperAddress.dynamic_addressing(0,
                                            n_digital_inputs,
                                            lambda ix: ros_wrapper.digital_read(dios[ix]) == PinState.HIGH),
        100:  # freedrive / learning mode
        DigitalWrapperAddress(read=lambda: ros_wrapper.get_learning_mode()),
        110:  # calibration needed
        DigitalWrapperAddress(read=lambda: ros_wrapper.get_hardware_status().calibration_needed),
        111:  # calibration in progress
        DigitalWrapperAddress(read=lambda: ros_wrapper.get_hardware_status().calibration_in_progress),
        112:  # motor connection ok
        DigitalWrapperAddress(read=lambda: ros_wrapper.get_hardware_status().connection_up),
        # conveyor bus ok
        **DigitalWrapperAddress.dynamic_addressing(200,
                                            n_conveyors,
                                            lambda ix: ros_wrapper.get_conveyors_feedback()[ix].connection_state),
        # conveyor is running
        **DigitalWrapperAddress.dynamic_addressing(210,
                                            n_conveyors,
                                            lambda ix: ros_wrapper.get_conveyors_feedback()[ix].running),
        # conveyor direction
        **DigitalWrapperAddress.dynamic_addressing(220,
                                            n_conveyors,
                                            lambda ix: ros_wrapper.get_conveyors_feedback()[ix].direction == 1),
    }
