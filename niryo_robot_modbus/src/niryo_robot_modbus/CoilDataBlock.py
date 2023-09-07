#!/usr/bin/env python
import rospy
from collections import OrderedDict

from .data_block import SequentialDataBlock

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState

from niryo_robot_rpi.srv import SetIOMode, SetIOModeRequest, SetDigitalIO


class CoilDataBlock(SequentialDataBlock):
    """
    Coil: Read / Write binary data
    """
    IO_MODE_START_ADDR = 0
    IO_STATE_START_ADDR = 100
    CUSTOM_VAR_START_ADDR = 200

    if rospy.get_param("/niryo_robot_modbus/hardware_version") == "ned2":
        DIO_ADDRESS = OrderedDict({
            0: "DI1",
            1: "DI2",
            2: "DI3",
            3: "DI4",
            4: "DI5",
            5: "DO1",
            6: "DO2",
            7: "DO3",
            8: "DO4",
        })
    else:
        DIO_ADDRESS = OrderedDict({
            0: "1A",
            1: "1B",
            2: "1C",
            3: "2A",
            4: "2B",
            5: "2C",
            6: "SW1",
            7: "SW2",
        })

    DIO_MODE_OUTPUT = SetIOModeRequest.OUTPUT
    DIO_MODE_INPUT = SetIOModeRequest.INPUT

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self.__ros_wrapper = ros_wrapper
        super().__init__(values=[False] * 299)

    # Override
    def setValues(self, address, values):
        if address >= self.CUSTOM_VAR_START_ADDR:
            super().setValues(address, values)
        elif address >= self.IO_STATE_START_ADDR:
            pin_id = self.DIO_ADDRESS[address % self.IO_STATE_START_ADDR]
            self.__ros_wrapper.digital_write(pin_id, values[0])
        elif address >= self.IO_MODE_START_ADDR:
            pin_id = self.DIO_ADDRESS[address % self.IO_MODE_START_ADDR]
            self.__ros_wrapper.set_pin_mode(pin_id, values[0])

    def getValues(self, address, count=1):
        if address >= self.CUSTOM_VAR_START_ADDR:
            return super().getValues(address)
        elif address >= self.IO_STATE_START_ADDR:
            pin_id = self.DIO_ADDRESS[address % self.IO_STATE_START_ADDR]
            dio_states = self.__ros_wrapper.get_digital_io_state()
            return pin_id in [dio.name for dio in dio_states.digital_inputs]
        elif address >= self.IO_MODE_START_ADDR:
            pin_id = self.DIO_ADDRESS[address % self.IO_MODE_START_ADDR]
            return self.__ros_wrapper.digital_read(pin_id) == PinState.HIGH
