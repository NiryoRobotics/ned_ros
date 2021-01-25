#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Bool
from enum import IntEnum

from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_rpi.msg import DigitalIOState
from niryo_robot_rpi.srv import GetDigitalIO
from niryo_robot_rpi.srv import SetDigitalIO


class FakeNiryoButton:
    def __init__(self):
        # Publisher used to send info to Niryo Studio, so the user can add a move block
        # by pressing the button
        self.save_point_publisher = rospy.Publisher(
            "/niryo_robot/blockly/save_current_point", Int32, queue_size=10)

        self.__button_state_publisher = rospy.Publisher(
            "/niryo_robot/rpi/is_button_pressed", Bool, latch=True, queue_size=1)

        self.__button_state_publisher.publish(False)


GPIO_1_A = 2
GPIO_1_B = 3
GPIO_1_C = 16
GPIO_2_A = 26
GPIO_2_B = 19
GPIO_2_C = 6

SW_1 = 12
SW_2 = 13

GPIO_1_A_NAME = '1A'
GPIO_1_B_NAME = '1B'
GPIO_1_C_NAME = '1C'
GPIO_2_A_NAME = '2A'
GPIO_2_B_NAME = '2B'
GPIO_2_C_NAME = '2C'

SW_1_NAME = 'SW1'
SW_2_NAME = 'SW2'


class FakeDigitalPinMode(IntEnum):
    OUTPUT = 0
    INPUT = 1


class FakeDigitalPin:
    def __init__(self, pin, name, mode=FakeDigitalPinMode.INPUT, state=0):
        self.pin = pin
        self.name = name
        self.state = state
        self.mode = mode

    def set_mode(self, mode):
        self.mode = mode
        if mode == FakeDigitalPinMode.OUTPUT:
            self.set_state(0)

    def get_mode(self):
        return self.mode

    def set_state(self, state):
        if self.mode != FakeDigitalPinMode.OUTPUT:
            return False
        self.state = state
        return True

    def get_state(self):
        return self.state

    def __str__(self):
        return self.name


class FakeDigitalIOPanel:

    def __init__(self):
        self.publish_io_state_frequency = rospy.get_param("~publish_io_state_frequency")
        self.digitalIOs = [FakeDigitalPin(GPIO_1_A, GPIO_1_A_NAME),
                           FakeDigitalPin(GPIO_1_B, GPIO_1_B_NAME),
                           FakeDigitalPin(GPIO_1_C, GPIO_1_C_NAME),
                           FakeDigitalPin(GPIO_2_A, GPIO_2_A_NAME),
                           FakeDigitalPin(GPIO_2_B, GPIO_2_B_NAME),
                           FakeDigitalPin(GPIO_2_C, GPIO_2_C_NAME),
                           FakeDigitalPin(SW_1, SW_1_NAME, mode=FakeDigitalPinMode.OUTPUT),
                           FakeDigitalPin(SW_2, SW_2_NAME, mode=FakeDigitalPinMode.OUTPUT)]

        self.digital_io_publisher = rospy.Publisher('/niryo_robot_rpi/digital_io_state', DigitalIOState, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.publish_io_state_frequency), self.publish_io_state)

        self.get_io_server = rospy.Service('/niryo_robot_rpi/get_digital_io', GetDigitalIO, self.callback_get_io)
        self.set_io_mode_server = rospy.Service('/niryo_robot_rpi/set_digital_io_mode', SetDigitalIO,
                                                self.callback_set_io_mode)
        self.set_io_state_server = rospy.Service('/niryo_robot_rpi/set_digital_io_state', SetDigitalIO,
                                                 self.callback_set_io_state)

    def publish_io_state(self, event):
        msg = DigitalIOState()
        pins = []
        names = []
        modes = []
        states = []
        for io in self.digitalIOs:
            pins.append(io.pin)
            names.append(io.name)
            modes.append(io.mode)
            states.append(io.get_state())
        msg.pins = pins
        msg.names = names
        msg.modes = modes
        msg.states = states
        try:
            self.digital_io_publisher.publish(msg)
        except rospy.ROSException:
            return

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def callback_get_io(self, req):
        for io in self.digitalIOs:
            if io.pin == req.pin:
                return {
                    'status': CommandStatus.SUCCESS,
                    'message': 'OK',
                    'pin': io.pin,
                    'name': io.name,
                    'mode': io.get_mode(),
                    'state': io.get_state()
                }
        return self.create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                    "No GPIO found with this pin number (" + str(req.pin) + ")")

    def callback_set_io_mode(self, req):
        for io in self.digitalIOs:
            if io.pin == req.pin:
                if io.name.startswith('SW'):
                    return self.create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                                "Can't change mode for switch pin, mode is fixed to OUTPUT")
                # Set mode
                if req.value == 0:
                    io.set_mode(FakeDigitalPinMode.OUTPUT)
                else:
                    io.set_mode(FakeDigitalPinMode.INPUT)
                return self.create_response(CommandStatus.SUCCESS, "Successfully set IO mode for pin " + str(io))
        # No pin found
        return self.create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                    "No GPIO found with this pin number (" + str(req.pin) + ")")

    def callback_set_io_state(self, req):
        for io in self.digitalIOs:
            if io.pin == req.pin:
                # Check gpio in in output mode
                if io.get_mode() != FakeDigitalPinMode.OUTPUT:
                    return self.create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR, "The pin " + str(
                        io) + " is set as input, you can't change its state")

                # Set state
                success = False
                if req.value == 0:
                    success = io.set_state(0)
                else:
                    success = io.set_state(1)

                if success:
                    return self.create_response(CommandStatus.SUCCESS,
                                                "Successfully set IO state for pin " + str(io))
                else:
                    return self.create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                                "Error : could not set IO state for pin " + str(io))

        # No pin found
        return self.create_response(CommandStatus.DIGITAL_IO_PANEL_ERROR,
                                    "No GPIO found with this pin number (" + str(req.pin) + ")")


class NiryoFakeRpi:

    def __init__(self):
        self.niryo_robot_fake_button = FakeNiryoButton()
        self.digital_io_panel = FakeDigitalIOPanel()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_rpi')
    NiryoFakeRpi()
    rospy.spin()
