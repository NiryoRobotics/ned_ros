from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState, PinMode
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper

from .WrapperDataBlock import WrapperDataBlock
from niryo_robot_modbus.src.niryo_robot_modbus.WrapperAddress import DigitalWrapperAddress


class CoilDataBlock(WrapperDataBlock):

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        super().__init__(ros_wrapper)

        digital_io = self._ros_wrapper.get_digital_io_state()
        self._bidirectional_ios = self._ros_wrapper.get_hardware_version() == 'ned'
        self._digital_outputs = [do.name for do in digital_io.digital_outputs]
        if self._bidirectional_ios:
            # ned IOs can be set either in input mode or output mode
            # we put them to the front of the list in order to keep the legacy addresses order
            self._digital_outputs = [di.name for di in digital_io.digital_inputs] + self._digital_outputs

    def _get_addressing(self):
        addressing = {
            # Digital io states
            **DigitalWrapperAddress.dynamic_addressing(
                0,
                len(self._digital_outputs),
                read=lambda ix: self._ros_wrapper.digital_read(self._digital_outputs[ix]) == PinState.HIGH,
                write=(lambda ix,
                       value: self._ros_wrapper.digital_write(self._digital_outputs[ix],
                                                              PinState.HIGH if value[0] else PinState.LOW)),
            ),
            **{
                a: None
                for a in range(200, 300)
            }
        }
        if self._bidirectional_ios:
            addressing.update(
                # Digital io modes
                DigitalWrapperAddress.dynamic_addressing(
                    100,
                    len(self._digital_outputs),
                    read=(lambda ix: self._digital_outputs[ix] in
                          [x.name for x in self._ros_wrapper.get_digital_io_state().digital_inputs]),
                    write=(lambda ix,
                           value: self._ros_wrapper.set_pin_mode(self._digital_outputs[ix],
                                                                 PinMode.INPUT if value[0] else PinMode.OUTPUT))))

        return addressing
