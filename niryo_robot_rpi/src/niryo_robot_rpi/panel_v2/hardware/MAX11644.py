# -----------------------------------------------------------
# Library to control MAX11644 device
#
# (C) 2021 Christopher Sanchez, Lille, France
# Released under XX License
# email c.sanchez@niryo.com
# ---

from smbus2 import SMBus
import time

__version__ = "1.1.0-auto.0"
# Internal constants:
_DEFAULT_ADDRESS = 0x36
_DEFAULT_BUS = 1
_V_REF_INTERNAL = 4.096
_VDD = 5.0

"""
    MAX11644 is 12-bit analog to digital converter.
    :param int bus: The channel number if it's different to 1.
    :param int address: The address of the device if set differently from the default
    :param bool internal_reference: External or internal voltage reference boolean (True, False if you want to use VDD
            as reference)
"""


class MAX11644(object):
    def __init__(self, bus=_DEFAULT_BUS, address=_DEFAULT_ADDRESS, internal_reference=True,
                 v_ref_internal=_V_REF_INTERNAL):
        # I2C bus object
        self._i2c = SMBus(bus)
        # added to avoid communication problems
        time.sleep(1)
        self._address = address
        self._bus = bus
        self._SETUP = 0x80
        self._CONF = 0x00

        if internal_reference:
            self._SETUP = 0xD0
            self._ref_volt = v_ref_internal
        else:
            self._SETUP = 0x82
            self._ref_volt = _VDD

        self._SETUP = (self._SETUP | 0x02)  # internal clock, unipolar, no action
        self._send_data_(self._SETUP)

    def __str__(self):
        return 'MAX11644, bus {}, address {}'.format(self._bus, self._address)

    @property
    def address(self):
        return self._address

    @property
    def bus(self):
        return self._bus

    def _send_data_(self, byte):
        """Send a byte to device"""
        self._i2c.write_byte(self._address, byte)
        time.sleep(0.1)

    def _read_data(self, address):
        """Read 2 bytes from address address."""
        data = self._i2c.read_i2c_block_data(self._address, address, 2)
        return data

    def get_value(self, channel=0):
        """Get Full scale value (12 bits)"""
        if channel == 0:
            self._CONF = 0x61
            data = self._read_data(self._CONF)

        elif channel == 1:
            self._CONF = 0x63
            data = self._read_data(self._CONF)
        else:
            raise Exception("Channel doesn't exist")
        data = ((data[0] & 0x0F) << 8) | data[1]
        return data

    def convert_to_volt(self, value):
        """Convert Full scale value (12 bits) into volts"""
        return self._ref_volt * value / 4096.

    def get_voltage(self, channel=0, factor=1):
        return self.convert_to_volt(self.get_value(channel) / factor)
