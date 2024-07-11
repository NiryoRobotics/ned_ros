"""
Set of classes which allow to manage GPIO pins as if they were MCP expander's pins

This module is meant to grow depending on the needs to mimic the MCP manager
"""
from RPi import GPIO


class GpioExpanderAdapter:

    def __init__(self, pin, mode):
        self._pin = pin
        self._mode = mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, self._mode)


class GpioExpanderInputAdapter(GpioExpanderAdapter):

    def __init__(self, pin):
        super().__init__(pin, GPIO.IN)

    @property
    def value(self):
        return GPIO.input(self._pin)

    def on_change(self, fun):
        GPIO.add_event_detect(self._pin, GPIO.BOTH, callback=lambda _pin: fun(self.value), bouncetime=10)


class GpioExpanderOutputAdapter(GpioExpanderAdapter):

    def __init__(self, pin):
        super().__init__(pin, GPIO.OUT)
        self.value = False

    @property
    def value(self):
        return GPIO.input(self._pin)

    @value.setter
    def value(self, value):
        GPIO.output(self._pin, GPIO.HIGH if value else GPIO.LOW)


class GpioExpanderButtonAdapter(GpioExpanderInputAdapter):

    def on_press(self, fun):
        GPIO.add_event_detect(self._pin, GPIO.RISING, callback=lambda _pin: fun(), bouncetime=10)

    def disable_on_press(self):
        GPIO.remove_event_detect(self._pin)


class GpioManager:

    def __init__(self):
        self.__inputs = {}
        self.__outputs = {}

    def add_output(self, pin, name, reversed=False):
        self.__outputs[name] = GpioExpanderOutputAdapter(pin)
        return self.__outputs[name]

    def add_button(self, pin, name, pullup=True, reverse_polarity=False):
        self.__inputs[name] = GpioExpanderButtonAdapter(pin)
        return self.__inputs[name]

    def add_input(self, pin, name, reverse_polarity=True):
        self.__inputs[name] = GpioExpanderInputAdapter(pin)
        return self.__inputs[name]
