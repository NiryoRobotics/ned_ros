from typing import Dict, List

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, PinState, PinMode

from .WrapperAddress import WrapperAddress


def get_digital_io_ids(ros_wrapper: NiryoRosWrapper) -> List[str]:
    """
    Get the list of the robot's digital output IDs.

    Returns:
        List[str]: A list of digital output IDs.
    """
    digital_io = ros_wrapper.get_digital_io_state()
    digital_outputs = [do.name for do in digital_io.digital_outputs]
    if ros_wrapper.get_hardware_version() == 'ned':
        # ned IOs can be set either in input mode or output mode
        # we put them to the front of the list in order to keep the legacy addresses order
        digital_outputs = [di.name for di in digital_io.digital_inputs] + digital_outputs
    return digital_outputs


def get_addressing(ros_wrapper: NiryoRosWrapper) -> Dict[int, WrapperAddress]:
    """
    This function creates a dynamic addressing scheme for reading and writing on coil data blocks.
    The addressing scheme is designed to handle digital output states for all available digital I/O pins.
    For the ned, it also supports digital input states and modes.

    :param ros_wrapper: An instance of the NiryoRosWrapper class providing ROS interface.
    :type ros_wrapper: NiryoRosWrapper
    :returns: A dictionary mapping addresses to WrapperAddress objects.
    :rtype: Dict[int, WrapperAddress]
    """
    hardware_version = ros_wrapper.get_hardware_version()
    dios = get_digital_io_ids(ros_wrapper)
    n_dios = len(dios)

    addressing = {
        # Digital io states
        **WrapperAddress.dynamic_addressing(
            0,
            n_dios,
            read=lambda ix: ros_wrapper.digital_read(dios[ix]) == PinState.HIGH,
            write=lambda ix,
            value: ros_wrapper.digital_write(dios[ix], PinState.HIGH if value[0] else PinState.LOW),
        ),
    }
    if hardware_version == 'ned':
        addressing.update(
            # Digital io modes
            WrapperAddress.dynamic_addressing(
                100,
                n_dios,
                read=lambda ix: dios[ix] in [x.name for x in ros_wrapper.get_digital_io_state().digital_inputs],
                write=(
                    lambda ix, value: ros_wrapper.set_pin_mode(dios[ix], PinMode.INPUT
                                                               if value[0] else PinMode.OUTPUT))))

    return addressing
