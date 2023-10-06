from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper

from .WrapperDataBlock import WrapperDataBlock
from .WrapperAddress import DigitalWrapperAddress
"""
Coil:
xx stop move
xx linear mode

xx*2 scan and attach conveyor
xx*2 detach conveyor

xx*2 conveyor enabled
xx*2 conveyor direction (0 = backward, 1 = forward)

xx freedrive / learning mode
xx calibration needed
xx auto calibration
xx scan tool
xx open gripper
xx close gripper
xx pull vacuum pump
xx push vacuum pump
xx tcp transformation enabled
"""


class CoilDataBlock(WrapperDataBlock):

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        digital_io = ros_wrapper.get_digital_io_state()
        self._digital_outputs = [do.name for do in digital_io.digital_outputs]
        super().__init__(ros_wrapper)

    def _get_addressing(self):
        return {
            # Digital io states
            **DigitalWrapperAddress.dynamic_addressing(
                0,
                len(self._digital_outputs),
                read=lambda ix: self._ros_wrapper.digital_read(self._digital_outputs[ix]) == PinState.HIGH,
                write=(lambda ix,
                       value: self._ros_wrapper.digital_write(self._digital_outputs[ix],
                                                              PinState.HIGH if value[0] else PinState.LOW)),
            ),  # User store
            **DigitalWrapperAddress.dynamic_addressing(200, 99),
        }
