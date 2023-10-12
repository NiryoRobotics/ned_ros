from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState, ConveyorDirection
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_status.msg import RobotStatus

from .WrapperDataBlock import WrapperDataBlock
from .WrapperAddress import DigitalWrapperAddress

from tools_interface.msg import Tool
"""
Coil:
0-50 DO state
51 scan tool
52 tool id
53 open gripper
54 close gripper
55 pull vacuum pump
56 push vacuum pump
57 tcp transformation enabled
60*2 scan and attach conveyor
62*2 detach conveyor
64*2 conveyor enabled
66*2 conveyor direction (0 = backward, 1 = forward)
70 stop move
71 linear mode
72 freedrive / learning mode
73 calibration needed
74 auto calibration
200-299 user store
"""


class CoilDataBlock(WrapperDataBlock):

    def __init__(self, ros_wrapper: NiryoRosWrapper, shared_data):
        digital_io = ros_wrapper.get_digital_io_state()
        self._digital_outputs = [do.name for do in digital_io.digital_outputs]
        super().__init__(ros_wrapper, shared_data)

    def _get_addressing(self):
        return {
            # Digital io states
            **DigitalWrapperAddress.dynamic_addressing(
                0,
                len(self._digital_outputs),
                read=lambda ix: self._ros_wrapper.digital_read(self._digital_outputs[ix]) == PinState.HIGH,
                write=(lambda ix, value: self._ros_wrapper.digital_write(self._digital_outputs[ix],
                                                                         PinState.HIGH if value[0] else PinState.LOW)),
            ),
            51:  # scan tool
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_current_tool_id(),
                                  write=lambda value: self._ros_wrapper.update_tool()),
            52:  # actuate tool
            DigitalWrapperAddress(
                read=lambda: self._ros_wrapper.get_current_tool_state() in [Tool.GRIPPER_STATE_CLOSE,
                                                                            Tool.VACUUM_PUMP_STATE_PULLED],
                write=lambda v: self._ros_wrapper.grasp_with_tool() if v else self._ros_wrapper.release_with_tool()
            ),
            53:  # tcp enabled
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_tcp().enabled,
                                  write=lambda v: self._ros_wrapper.enable_tcp(v)),
            # conveyor is running
            **DigitalWrapperAddress.dynamic_addressing(
                60,
                2,
                read=lambda ix: self._safe_conveyor_feedback(ix).running,
                write=lambda ix, v: self._safe_control_conveyor(ix, bool_control_on=v)
            ),
            # conveyor direction
            **DigitalWrapperAddress.dynamic_addressing(
                62,
                2,
                read=lambda ix: self._safe_conveyor_feedback(ix).direction == ConveyorDirection.FORWARD,
                write=lambda ix, v: self._safe_control_conveyor(
                    ix,
                    direction=ConveyorDirection.FORWARD if v else ConveyorDirection.BACKWARD
                )
            ),
            70:  # robot in movement
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_robot_status().robot_status == RobotStatus.MOVING,
                                  write=lambda v: self._ros_wrapper.stop_move() if not v else None),
            71:
            DigitalWrapperAddress(read=lambda: self._shared_data.linear_mode,
                                  write=lambda v: self._shared_data.set_linear_mode(v)),
            72:  # freedrive / learning mode
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_learning_mode(),
                                  write=lambda v: self._ros_wrapper.set_learning_mode(v)),
            73:  # calibration needed
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().calibration_needed,
                                  write=lambda v: self._ros_wrapper.request_new_calibration() if v else None),
            74:  # calibration
            DigitalWrapperAddress(read=lambda: self._ros_wrapper.get_hardware_status().calibration_in_progress,
                                  write=lambda v: self._ros_wrapper.calibrate_auto() if v else None),
            # User store
            **DigitalWrapperAddress.dynamic_addressing(200, 99),
        }
