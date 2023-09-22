#!/usr/bin/env python3

from typing import List

from pymodbus.datastore import ModbusSparseDataBlock

import rospy
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import PinState

from .utils import address_range


class DiscreteInputDataBlock(ModbusSparseDataBlock):
    """
    Discrete Input: Read Only binary data
    """

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self.__ros_wrapper = ros_wrapper

        self.__digital_inputs_ids = self.get_digital_inputs_ids()

        super().__init__({address: False for address in address_range(len(self.__digital_inputs_ids))})

    def get_digital_inputs_ids(self) -> List[str]:
        """
        Get the list of the robot's digital output IDs.

        Returns:
            List[str]: A list of digital output IDs.
        """
        if self.__ros_wrapper.get_hardware_version() == 'ned':
            # All Ned's IOs are in Coils since they are either output only or input / output
            return []

        digital_io = self.__ros_wrapper.get_digital_io_state()
        digital_inputs = [di.name for di in digital_io.digital_inputs]
        return digital_inputs

    # Override ModbusSparseDataBlock

    def getValues(self, address: int, count: int = 1) -> List[bool]:
        pin_id = self.__digital_inputs_ids[address]
        return [self.__ros_wrapper.digital_read(pin_id) == PinState.HIGH]
