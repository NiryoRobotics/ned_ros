import rospy

from niryo_robot_python_ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState

from .WrapperDataBlock import WrapperDataBlock
from .WrapperAddress import DigitalWrapperAddress


class DiscreteInputDataBlock(WrapperDataBlock):

    def __init__(self, ros_wrapper: NiryoRosWrapper, shared_data):
        self._dios = [di.name for di in ros_wrapper.get_digital_io_state().digital_inputs]
        super().__init__(ros_wrapper, shared_data)

    def _get_addressing(self):
        n_digital_inputs = len(self._ros_wrapper.get_digital_io_state().digital_inputs)
        n_can_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/can/pool_id_list', []))
        n_ttl_conveyors = len(rospy.get_param('/niryo_robot_hardware_interface/conveyor/ttl/pool_id_list', []))
        n_conveyors = n_can_conveyors + n_ttl_conveyors
        return {
            # Digital inputs states
            **DigitalWrapperAddress.dynamic_addressing(
                0,
                n_digital_inputs,
                lambda ix: self._ros_wrapper.digital_read(self._dios[ix]) == PinState.HIGH
            ),

            50:  # motor connection ok
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().connection_up),
            # conveyor bus ok
            **DigitalWrapperAddress.dynamic_addressing(51,
                                                       n_conveyors,
                                                       lambda ix: self._safe_conveyor_feedback(ix).connection_state),
            52:  # is executing command
            DigitalWrapperAddress(read=lambda: self._shared_data.is_executing_command),
            53:  # vision target found
            DigitalWrapperAddress(read=lambda: self._get_vision_target()['found']),
        }
