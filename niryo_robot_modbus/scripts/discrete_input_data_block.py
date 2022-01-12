#!/usr/bin/env python

import rospy
from collections import OrderedDict

from data_block import DataBlock

from niryo_robot_rpi.msg import DigitalIOState, AnalogIOState
from niryo_robot_rpi.srv import SetIOModeRequest

"""
 - Each address contains a 1 bit value
 - READ ONLY registers

 --> State of the robot
"""


class DiscreteInputDataBlock(DataBlock):
    DI_IO_MODE = 0
    DI_IO_STATE = 100

    if rospy.get_param("/niryo_robot_modbus/hardware_version") == "ned2":
        DIO_ADDRESS = OrderedDict({"DI1": 0,
                                   "DI2": 1,
                                   "DI3": 2,
                                   "DI4": 3,
                                   "DI5": 4,
                                   "DO1": 5,
                                   "DO2": 6,
                                   "DO3": 7,
                                   "DO4": 8, })
    else:
        DIO_ADDRESS = OrderedDict({"1A": 0,
                                   "1B": 1,
                                   "1C": 2,
                                   "2A": 3,
                                   "2B": 4,
                                   "2C": 5,
                                   "SW1": 6,
                                   "SW2": 7, })

    DIO_MODE_OUTPUT = SetIOModeRequest.OUTPUT
    DIO_MODE_INPUT = SetIOModeRequest.INPUT

    def __init__(self):
        super(DiscreteInputDataBlock, self).__init__()
        self._digital_io_state_sub = None

    def start_ros_subscribers(self):
        self._digital_io_state_sub = rospy.Subscriber('/niryo_robot_rpi/digital_io_state', DigitalIOState,
                                                      self._sub_digital_io_state)

    def stop_ros_subscribers(self):
        self._digital_io_state_sub.unregister()

    def _sub_digital_io_state(self, msg):
        for din in msg.digital_inputs:
            self.setValuesOffset(self.DI_IO_MODE + self.DIO_ADDRESS[din.name], self.DIO_MODE_INPUT)
            self.setValuesOffset(self.DI_IO_STATE + self.DIO_ADDRESS[din.name], int(din.value))
        for dout in msg.digital_outputs:
            self.setValuesOffset(self.DI_IO_MODE + self.DIO_ADDRESS[dout.name], self.DIO_MODE_OUTPUT)
            self.setValuesOffset(self.DI_IO_STATE + self.DIO_ADDRESS[dout.name], int(dout.value))
