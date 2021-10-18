import rospy

from niryo_robot_rpi.common.io_objects import PinMode, NiryoIO, NiryoIOException

from end_effector_interface.srv import SetEEDigitalOut


class DigitalOutput(NiryoIO):
    def __init__(self, name):
        super(DigitalOutput, self).__init__(lock=None, pin=0, name=name)

        self.__set_ee_io_state_service = rospy.ServiceProxy(
            "/niryo_robot_hardware_interface/end_effector_interface/set_ee_io_state", SetEEDigitalOut)

        self.value = False

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        self.__set_ee_io_state_service(bool(value))
        self._value = bool(value)


class DigitalInput(NiryoIO):
    def __init__(self, name):
        super(DigitalInput, self).__init__(lock=None, pin=0, name=name)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value
