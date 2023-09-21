#!/usr/bin/env python3

from typing import List

from pymodbus.datastore import ModbusSparseDataBlock

import rospy

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinMode, PinState

from .utils import address_range


class CoilDataBlock(ModbusSparseDataBlock):
    """
    Coil: Read / Write binary data

    The coil datas will contain addresses for accessing the digital output states of the robot.
    Each feature is separated by an offset.
    For instance, address 003 is used to access the state of D04, and address 103 is used to access its mode.

    state = HIGH (1) / LOW (0)
    mode = INPUT (1) / OUTPUT (0)
    """
    DO_MODE_OFFSET_ADDRESS = rospy.get_param('/niryo_robot_modbus/register_addresses_offset/coils/do_mode')
    USER_STORE_OFFSET_ADDRESS = rospy.get_param('/niryo_robot_modbus/register_addresses_offset/coils/user_store')

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self.__ros_wrapper = ros_wrapper
        self.__has_bidirectional_ios = self.__ros_wrapper.get_hardware_version() == 'ned'

        self.__digital_outputs_ids = self.get_digital_outputs_ids()

        super().__init__({address: False for address in self.get_addresses()})

    def get_addresses(self) -> List[int]:
        """
        Get a list of Modbus addresses for the digital outputs and modes.

        Returns:
            List[int]: A list of Modbus addresses.
        """
        n_digital_outputs = len(self.__digital_outputs_ids)

        # digital output state addresses
        addresses = address_range(n_digital_outputs)

        if self.__has_bidirectional_ios:
            # digital i/o mode addresses
            addresses += address_range(n_digital_outputs, self.DO_MODE_OFFSET_ADDRESS)
        # user store addresses
        addresses += address_range(100, self.USER_STORE_OFFSET_ADDRESS)
        return addresses

    def get_digital_outputs_ids(self) -> List[str]:
        """
        Get the list of the robot's digital output IDs.

        Returns:
            List[str]: A list of digital output IDs.
        """
        digital_io = self.__ros_wrapper.get_digital_io_state()
        digital_outputs = [do.name for do in digital_io.digital_outputs]
        if self.__ros_wrapper.get_hardware_version() == 'ned':
            # ned IOs can be set either in input mode or output mode
            # we put them to the front of the list in order to keep the legacy addresses order
            digital_outputs = [di.name for di in digital_io.digital_inputs] + digital_outputs
        return digital_outputs

    def get_pin_id_from_address(self, address: int) -> str:
        """
        Get the digital output pin ID from a Modbus address.

        Args:
            address (int): The Modbus address.

        Returns:
            str: The digital output pin ID.
        """
        for offset in sorted([self.DO_MODE_OFFSET_ADDRESS], reverse=True):
            if address >= offset:
                return self.__digital_outputs_ids[address % offset]
        return self.__digital_outputs_ids[address]

    # Override ModbusSparseDataBlock
    def setValues(self, address: int, values: List[bool]) -> None:
        if address >= self.USER_STORE_OFFSET_ADDRESS:
            super().setValues(address, values)
            return

        pin_id = self.get_pin_id_from_address(address)
        if address >= self.DO_MODE_OFFSET_ADDRESS:
            self.__ros_wrapper.set_pin_mode(pin_id, values[0])
        else:
            if self.__ros_wrapper.get_digital_io_mode(pin_id) == PinMode.INPUT:
                return
            self.__ros_wrapper.digital_write(pin_id, values[0])

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        if address >= self.USER_STORE_OFFSET_ADDRESS:
            return super().getValues(address)

        pin_id = self.get_pin_id_from_address(address)
        if address >= self.DO_MODE_OFFSET_ADDRESS:
            return [self.__ros_wrapper.get_digital_io_mode(pin_id) == PinMode.INPUT]
        else:
            return [self.__ros_wrapper.digital_read(pin_id) == PinState.HIGH]
