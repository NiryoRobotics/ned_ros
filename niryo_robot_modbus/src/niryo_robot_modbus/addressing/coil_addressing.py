import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, PinState

from . import dynamic_addressing


def get_addressing(ros_wrapper: NiryoRosWrapper):
    n_digital_outputs = len(ros_wrapper.get_digital_io_state().digital_outputs)

    return {
        # Digital output states
        **dynamic_addressing(0, n_digital_outputs, lambda ix: ros_wrapper.digital_write(ix)),
    }
