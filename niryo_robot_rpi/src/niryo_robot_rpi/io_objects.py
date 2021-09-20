import time
from threading import Thread


class PinMode:
    """
    Pin Mode is either OUTPUT or INPUT
    """

    def __init__(self):
        pass

    DIGITAL_OUTPUT = 0
    DIGITAL_INPUT = 1

    ANALOG_OUTPUT = 2
    ANALOG_INPUT = 3


class NiryoIOException(Exception):
    pass


class NiryoIO(object):

    def __init__(self, lock, pin, name):
        self._lock = lock
        self._pin = pin
        self._name = name
        self._mode = None
        self._value = 0

    @property
    def mode(self):
        return self._mode

    @property
    def pin(self):
        return self._pin

    @property
    def name(self):
        return self._name

    def __str__(self):
        return self._name
