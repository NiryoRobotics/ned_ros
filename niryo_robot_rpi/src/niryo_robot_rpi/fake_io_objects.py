import time
from io_objects import PinMode, NiryoIO


class FakeDigitalIO(NiryoIO):
    def __init__(self, name, mode):
        super(FakeDigitalIO, self).__init__(None, None, name)

        assert mode in [PinMode.DIGITAL_OUTPUT, PinMode.DIGITAL_INPUT], \
            "The pin mode of a fake digital io must be either PinMode.ANALOG_INPUT or PinMode.ANALOG_OUTPUT"

        self._mode = mode
        self._value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        limited_value = max(0, min(int(value), 1))
        self._value = limited_value


class FakeAnalogIO(NiryoIO):
    def __init__(self, name, mode):
        super(FakeAnalogIO, self).__init__(None, None, name)

        assert mode in [PinMode.ANALOG_INPUT, PinMode.ANALOG_OUTPUT], \
            "The pin mode of a fake analog io must be either PinMode.ANALOG_INPUT or PinMode.ANALOG_OUTPUT"

        self._mode = mode
        self._value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        self._value = value


class FakeRpiDigitalIO(NiryoIO):
    def __init__(self, name, mode):
        super(FakeRpiDigitalIO, self).__init__(None, None, name)

        assert mode in [PinMode.DIGITAL_OUTPUT, PinMode.DIGITAL_INPUT], \
            "The pin mode of a fake digital io must be either PinMode.ANALOG_INPUT or PinMode.ANALOG_OUTPUT"

        self._mode = mode
        self._value = 0

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert isinstance(value, (bool, int, float))
        limited_value = max(0, min(int(value), 1))
        self._value = bool(limited_value)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        assert mode in [PinMode.DIGITAL_OUTPUT, PinMode.DIGITAL_INPUT], \
            "The pin mode of a fake digital io must be either PinMode.ANALOG_INPUT or PinMode.ANALOG_OUTPUT"

        if mode != self._mode:
            self._value = 0
            self._mode = mode
