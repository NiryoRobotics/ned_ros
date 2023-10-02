import rospy

from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState

from conveyor_interface.msg import ConveyorFeedback

from . import safe_get
from .WrapperDataBlock import WrapperDataBlock
from niryo_robot_modbus.src.niryo_robot_modbus.WrapperAddress import DigitalWrapperAddress


class DiscreteInputDataBlock(WrapperDataBlock):

    def __init__(self, ros_wrapper):
        super().__init__(ros_wrapper)
        self._dios = [di.name for di in self._ros_wrapper.get_digital_io_state().digital_inputs]

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
            100:  # freedrive / learning mode
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_learning_mode()),
            110:  # calibration needed
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().calibration_needed),
            111:  # calibration in progress
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().calibration_in_progress),
            112:  # motor connection ok
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().connection_up),
            # conveyor bus ok
            **DigitalWrapperAddress.dynamic_addressing(
                200,
                n_conveyors,
                lambda ix: self._ros_wrapper.get_conveyors_feedback()[ix].connection_state,
            ),
            # conveyor is running
            **DigitalWrapperAddress.dynamic_addressing(
                210,
                n_conveyors,
                lambda ix: safe_get(self._ros_wrapper.get_conveyors_feedback(), ix, ConveyorFeedback()).running,
            ),
            # conveyor direction
            **DigitalWrapperAddress.dynamic_addressing(
                220,
                n_conveyors,
                lambda ix: self._ros_wrapper.get_conveyors_feedback()[ix].direction == 1),
        }
