#!/usr/bin/env python
import rospy

from data_block import DataBlock
from collections import OrderedDict

from niryo_robot_rpi.srv import SetDigitalIO, SetAnalogIO
from niryo_robot_rpi.srv import SetPullup, SetIOMode, SetIOModeRequest

"""
 - Each address contains a 1 bit value
 - READ/WRITE registers

 --> Used to give commands to the robot
 ( ! the stored values correspond to the last given command,
 not the current robot state !)
"""


class CoilDataBlock(DataBlock):
    CO_IO_MODE = 0
    CO_IO_STATE = 100

    if rospy.get_param("/niryo_robot_modbus/hardware_version") == "ned2":
        DIO_ADDRESS = OrderedDict({0: "DI1",
                                   1: "DI2",
                                   2: "DI3",
                                   3: "DI4",
                                   4: "DI5",
                                   5: "DO1",
                                   6: "DO2",
                                   7: "DO3",
                                   8: "DO4", })
    else:
        DIO_ADDRESS = OrderedDict({0: "1A",
                                   1: "1B",
                                   2: "1C",
                                   3: "2A",
                                   4: "2B",
                                   5: "2C",
                                   6: "SW1",
                                   7: "SW2", })

    DIO_MODE_OUTPUT = SetIOModeRequest.OUTPUT
    DIO_MODE_INPUT = SetIOModeRequest.INPUT

    def __init__(self):
        super(CoilDataBlock, self).__init__()

    # Override
    def setValues(self, address, values):
        self.process_command(address, values)
        super(CoilDataBlock, self).setValues(address, values)

    def process_command(self, address, values):
        address -= 1
        if len(values) == 0:
            return
        value = values[0]

        try:
            if address >= self.CO_IO_STATE:
                if address % 100 in self.DIO_ADDRESS:
                    self.digital_write(self.DIO_ADDRESS[address % 100], value)
            elif address >= self.CO_IO_MODE:
                pin_id = self.DIO_ADDRESS[address % 100]
                self.set_pin_mode(pin_id, value)
        except KeyError:
            pass

    def set_pin_mode(self, pin_id, pin_mode):
        """
        Set pin number pin_id to mode pin_mode

        :param pin_id:
        :type pin_id: PinID
        :param pin_mode:
        :type pin_mode: PinMode
        :return: status, message
        :rtype: None
        """
        _result = self.call_ros_service('/niryo_robot_rpi/set_digital_io_mode', SetIOMode, pin_id, pin_mode)

    def digital_write(self, pin_id, digital_state):
        """
        Set pin_id state to pin_state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :param digital_state:
        :type digital_state: Union[ PinState, bool]
        :rtype: None
        """
        _result = self.call_ros_service('/niryo_robot_rpi/set_digital_io', SetDigitalIO, pin_id,
                                        bool(digital_state))
