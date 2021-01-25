#!/usr/bin/env python

import rospy

from data_block import DataBlock

from niryo_robot_rpi.msg import DigitalIOState

"""
 - Each address contains a 1 bit value
 - READ ONLY registers

 --> State of the robot
"""

DI_DIGITAL_IO_MODE = 0
DI_DIGITAL_IO_STATE = 100


class DiscreteInputDataBlock(DataBlock):

    def __init__(self):
        super(DiscreteInputDataBlock, self).__init__()
        self.digital_io_state_sub = None

    def start_ros_subscribers(self):
        self.digital_io_state_sub = rospy.Subscriber('/niryo_robot_rpi/digital_io_state', DigitalIOState,
                                                     self.sub_digital_io_state)

    def stop_ros_subscribers(self):
        self.digital_io_state_sub.unregister()

    def sub_digital_io_state(self, msg):
        self.setValuesOffset(DI_DIGITAL_IO_MODE, list(msg.modes))
        self.setValuesOffset(DI_DIGITAL_IO_STATE, list(msg.states))
