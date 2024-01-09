from typing import List

import rospy
from pymodbus.exceptions import InvalidMessageReceivedException

from .WrapperDataBlock import WrapperDataBlock
"""
Holding register:
0-11 Joints target (mrad)
12-23 pose target xyz/rpy
24-35 tcp transformation
40 gripper open speed
41 gripper close speed
45 conveyor speed
50*n AO voltage (0v - 5000mv)
100-101 x_rel
102-103 y_rel
104-105 yaw_rel
106 height offset
107 wanted shape (-1: ANY, 1: CIRCLE, 2: SQUARE, 3: TRIANGLE)
108 wanted color (-1: ANY, 1: BLUE, 2: RED, 3: GREEN)
110-130 workspace's name
200-299: user store

"""


class HoldingRegisterDataBlock(WrapperDataBlock):

    def _get_addressing(self):
        return {0: None}

    def move_command(self, values):
        if len(values) != 7:
            raise InvalidMessageReceivedException('7 arguments have to be provided')
        rospy.loginfo(f'values: {values}')

    def setValues(self, address: int, values: List[bool]) -> None:
        rospy.loginfo(f'address {address}: values {values}')
