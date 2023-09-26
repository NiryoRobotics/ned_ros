import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, PinState

from . import dynamic_addressing


def get_addressing(ros_wrapper: NiryoRosWrapper):
    n_digital_inputs = len(ros_wrapper.get_digital_io_state().digital_inputs)
    n_can_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', []))
    n_ttl_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', []))
    n_conveyors = n_can_conveyors + n_ttl_conveyors

    return {
        # Digital inputs states
        **dynamic_addressing(0, n_digital_inputs, lambda ix: ros_wrapper.digital_read(ix) == PinState.HIGH),
        100:  # freedrive / learning mode
        lambda: ros_wrapper.get_learning_mode(),
        110:  # calibration needed
        lambda: ros_wrapper.get_hardware_status().calibration_needed,
        111:  # calibration in progress
        lambda: ros_wrapper.get_hardware_status().calibration_in_progress,
        112:  # motor connection ok
        lambda: ros_wrapper.get_hardware_status().connection_up,
        # conveyor bus ok
        **dynamic_addressing(200, n_conveyors, lambda ix: ros_wrapper.get_conveyors_feedback()[ix].connection_state),
        # conveyor is running
        **dynamic_addressing(210, n_conveyors, lambda ix: ros_wrapper.get_conveyors_feedback()[ix].running),
        # conveyor direction
        **dynamic_addressing(220, n_conveyors, lambda ix: ros_wrapper.get_conveyors_feedback()[ix].direction == 1),
    }  # yapf: disable
