# -----------------------------------------------------------
# Library to control DACx0501 devices
#
# (C) 2021 Christopher Sanchez, Lille, France
# Released under XX License
# email c.sanchez@niryo.com
# ---
import time

from smbus2 import SMBus

__version__ = "1.1.1-auto.0"

# Internal constants:
_ADC80501_DEFAULT_ADDRESS = 0x49
_channel_I2C = 1
_v_ref = 2.5
_resolution = 16
# offset
_NOOP = 0x00
_DEVID = 0x01
_SYNC = 0x02
_CONFIG = 0x03
_GAIN = 0x04
_TRIGGER = 0x05
_STATUS = 0x07
_DAC = 0x08

"""
    DAC80501 is 16-bit digital to analog converter.
    :param int bus: The channel number if it's different to 1.
    :param int address: The address of the device if set differently
                        from the default( see address in p.28 of datasheet ).
    :param float v_ref: External or internal voltage reference 2.5V, 3.3V or 5V
    :param int resolution: Resolution bits DAC80501 -> 16bits, DAC60501 -> 12 bits
"""


class DACx0501Exception(Exception):
    pass


class DACx0501(object):

    def __init__(self, bus=_channel_I2C, address=_ADC80501_DEFAULT_ADDRESS, v_ref=_v_ref, resolution=_resolution):

        # I2C bus object
        self._i2c = SMBus(bus)
        # added to avoid communication problems
        time.sleep(1)

        self.v_ref = v_ref
        self.res = resolution

        # I2C slave address
        self._address = address

        # reset configuration (SOFT-RESET)
        self._send_data_(_TRIGGER, 0x00, 0x0A)

        # enable asynchronus mode (for enabling synchronus mode(LDAC trigger) send 0x00, 0x01) p.32
        self._send_data_(_SYNC, 0x00, 0x00)

        # if vref different to 2.5V
        if self.v_ref == 2.5:
            # use internal Vref
            self._send_data_(_CONFIG, 0x00, 0x00)
            # set REF-DIV (Vref divided by 1 -> bit 8) to 0 and BUFF-gain (gain x2 -> bit 0) to 1.
            self._send_data_(_GAIN, 0x00, 0x01)
            self.vout_max = 5.0
        else:
            # use external Vref
            self._send_data_(_CONFIG, 0x01, 0x00)
            # set REF-DIV (Vref divided by 2 -> bit 8) to 1 and BUFF-gain (gain x2 -> bit 0) to 1.
            self._send_data_(_GAIN, 0x01, 0x01)
            self.vout_max = self.v_ref

    def _send_data_(self, addr, byte1, byte2):
        """Send byte1 and byte2 to addr address"""
        self._i2c.write_i2c_block_data(self._address, addr, [byte1, byte2])
        time.sleep(0.1)

    def _read_data(self, addr):
        """Read 2 bytes from address addr."""
        data = self._i2c.read_i2c_block_data(self._address, addr, 2)
        return data

    def set_voltage_(self, voltage):
        """Set a value, then calculate DAC value to send to device"""
        if not 0 <= voltage <= self.vout_max:
            raise DACx0501Exception("Voltage must be between 0 and Vout_max = {}".format(self.vout_max))

        dac_value = int((voltage * ((1 << self.res) - 1) / self.vout_max))

        if self.res == 12:
            byte1 = (dac_value >> 4)
            byte2 = (dac_value & 0x00F)
        elif self.res == 14:
            byte1 = (dac_value >> 6)
            byte2 = (dac_value & 0x03F)
        elif self.res == 16:
            byte1 = (dac_value >> 8)
            byte2 = (dac_value & 0x00FF)
        else:
            raise DACx0501Exception(
                "Invalid resolution. Actual resolution is {} and must be in [12, 14, 16]".format(self.res))

        self._send_data_(_DAC, byte1, byte2)
