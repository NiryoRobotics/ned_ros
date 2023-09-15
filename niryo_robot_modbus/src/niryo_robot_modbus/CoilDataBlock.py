#!/usr/bin/env python

import rospy
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.exceptions import ModbusIOException

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinMode


class CoilDataBlock(ModbusSparseDataBlock):
    """
    Coil: Read / Write binary data
    """
    IO_STATE_START_ADDR = 100
    CUSTOM_VAR_START_ADDR = 200

    if rospy.get_param("/niryo_robot_modbus/hardware_version") == "ned2":
        DIO_ADDRESS = {
            5: "DO1",
            6: "DO2",
            7: "DO3",
            8: "DO4",
        }
    else:
        DIO_ADDRESS = {
            0: "1A",
            1: "1B",
            2: "1C",
            3: "2A",
            4: "2B",
            5: "2C",
        }

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self.__ros_wrapper = ros_wrapper
        register_addresses = {k: False for k in self.DIO_ADDRESS.keys()}
        register_addresses.update({address + self.IO_STATE_START_ADDR: False for address in self.DIO_ADDRESS.keys()})
        register_addresses.update({address: False for address in range(200, 300)})
        super().__init__(register_addresses)

    # Override
    def setValues(self, address, values):
        if address >= self.CUSTOM_VAR_START_ADDR:
            super().setValues(address, values)
            return

        pin_id = self.DIO_ADDRESS[address % 100]
        if address >= self.IO_STATE_START_ADDR:
            if self.get_dio_mode(pin_id) == PinMode.INPUT:
                raise ModbusIOException("Can't change the state of an IO in input mode")
            self.__ros_wrapper.digital_write(pin_id, values[0])
        else:
            self.__ros_wrapper.set_pin_mode(pin_id, values[0])

    def getValues(self, address, count=1):
        if address >= self.CUSTOM_VAR_START_ADDR:
            return super().getValues(address)

        pin_id = self.DIO_ADDRESS[address % 100]
        if address >= self.IO_STATE_START_ADDR:
            return [self.get_dio_mode(pin_id)]
        else:
            return [self.__ros_wrapper.digital_read(pin_id)]

    def get_dio_mode(self, pin_id):
        dio_states = self.__ros_wrapper.get_digital_io_state()
        if pin_id in [dio.name for dio in dio_states.digital_inputs]:
            return PinMode.INPUT
        return PinMode.OUTPUT
