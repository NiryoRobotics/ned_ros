import sys
from enum import Enum


class RegistersMCP23017(Enum):
    IODIRA = 0x00
    IODIRB = 0x01
    IPOLA = 0x02
    IPOLB = 0x03
    GPINTENA = 0x04
    GPINTENB = 0x05
    DEFVALA = 0x06
    DEFVALB = 0x07
    INTCONA = 0x08
    INTCONB = 0x09
    IOCONA = 0x0A
    IOCONB = 0x0B
    GPPUA = 0x0C
    GPPUB = 0x0D
    INTFA = 0x0E
    INTFB = 0x0F
    INTCAPA = 0x10
    INTCAPB = 0x11
    GPIOA = 0x12
    GPIOB = 0x13
    OLATA = 0x14
    OLATB = 0x15


class MCP23017(object):
    OUTPUT = 0
    INPUT = 1

    def __init__(self, address, busnum=0):
        self.__address = address
        self.__bus = busnum
        self.__num_gpios = 16
        self.__direction = 0x00

        if sys.version[0] == '3':
            import Adafruit_GPIO.I2C as I2C
            self.i2c = I2C.Device(self.__address, self.__bus)
        else:
            from Adafruit_I2C import Adafruit_I2C
            self.i2c = Adafruit_I2C(self.__address, self.__bus)

        self.init_config()

    def __str__(self):
        return 'MCP23017, bus {}, address {}'.format(self.__bus, self.__address)

    @property
    def address(self):
        return self.__address

    @property
    def bus(self):
        return self.__bus

    def init_config(self):
        self.write8(RegistersMCP23017.IODIRA, 0b00000010)  # all outputs on port A
        self.write8(RegistersMCP23017.IODIRB, 0x00)  # all outputs on port B
        self.__direction = self.readU8(RegistersMCP23017.IODIRA)
        self.__direction |= self.readU8(RegistersMCP23017.IODIRB) << 8
        self.write8(RegistersMCP23017.GPPUA, 0x00)
        self.write8(RegistersMCP23017.GPPUB, 0x00)
        self.write8(RegistersMCP23017.IOCONA, 0b00000100)  # INTA as open-drain

    def config(self, pin, mode):
        if pin < 8:
            self.__direction &= 0xF0
            self.__direction |= self._read_and_change_pin(RegistersMCP23017.IODIRA, pin, mode)
        else:
            self.__direction &= 0x0F
            self.__direction |= self._read_and_change_pin(RegistersMCP23017.IODIRB, pin - 8, mode) << 8
        return self.__direction

    def output(self, pin, value):
        assert self.__direction & (1 << pin) == 0, "Pin %s not set to output" % pin

        if pin < 8:
            return self._read_and_change_pin(RegistersMCP23017.GPIOA, pin, value, self.readU8(RegistersMCP23017.OLATA))
        else:
            return self._read_and_change_pin(RegistersMCP23017.GPIOB, pin - 8, value,
                                             self.readU8(RegistersMCP23017.OLATB)) << 8

    def input(self, pin):
        assert 0 <= pin < self.__num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.__num_gpios)
        assert self.__direction & (1 << pin) != 0, "Pin %s not set to input" % pin

        if pin < 8:
            return self.readU8(RegistersMCP23017.GPIOA) & (1 << pin)
        else:
            return self.readU8(RegistersMCP23017.GPIOB) & (1 << pin - 8)

    def enable_interrupt(self, pin, enable):
        assert 0 <= pin < self.__num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.__num_gpios)

        if pin < 8:
            self._read_and_change_pin(RegistersMCP23017.GPINTENA, pin, int(enable))
        else:
            self._read_and_change_pin(RegistersMCP23017.GPINTENB, pin - 8, int(enable))

    def pullup(self, pin, value):
        if (pin < 8):
            return self._read_and_change_pin(RegistersMCP23017.GPPUA, pin, value)
        else:
            return self._read_and_change_pin(RegistersMCP23017.GPPUB, pin - 8, value) << 8 | self.readU8(
                RegistersMCP23017.GPPUA)

    def reverse_input_polarity(self, pin, reverse):
        value = bool(reverse)
        if (pin < 8):
            return self._read_and_change_pin(RegistersMCP23017.IPOLA, pin, value)
        else:
            return self._read_and_change_pin(RegistersMCP23017.IPOLB, pin - 8, value) << 8 | self.readU8(
                RegistersMCP23017.IPOLA)

    def read_all_gpios(self):
        value = self.readU8(RegistersMCP23017.GPIOA)
        value |= self.readU8(RegistersMCP23017.GPIOB) << 8
        return value

    def write8(self, register, value):
        assert isinstance(register, RegistersMCP23017), "The register must be a member of the RegistersMCP23017 enum."

        self.i2c.write8(register.value, value)

    def readU8(self, register):
        assert isinstance(register, RegistersMCP23017), "The register must be a member of the RegistersMCP23017 enum."

        return self.i2c.readU8(register.value)

    def debug(self):
        print(34 * "-")
        print("| {:^30s} |".format("MCP23017 registers BANK=0"))
        for name, member in RegistersMCP23017.__members__.items():
            print(34 * "-")
            print("| {:^10s} | 0x{:02X} | 0b{:08b} |".format(name, member.value, self.readU8(member)))
        print(34 * "-")

    @staticmethod
    def _change_bit(bitmap, bit, value):
        assert value == 1 or value == 0, "Value is %s must be 1 or 0" % value
        if value == 1:
            return bitmap | (1 << bit)
        else:
            return bitmap & ~(1 << bit)

    def _read_and_change_pin(self, port, pin, value, current_value=None):
        assert pin <= pin < self.__num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.__num_gpios)

        if not current_value:
            current_value = self.readU8(port)

        new_value = self._change_bit(current_value, pin, value)
        self.write8(port, new_value)
        return new_value
