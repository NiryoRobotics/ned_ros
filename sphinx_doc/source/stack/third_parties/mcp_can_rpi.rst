MCP CAN rpi
===================================

Raspberry Pi library for MCP2515 module (CAN bus interface) through SPI GPIOs

Forked from `[MCP_CAN] library <https://github.com/coryjfowler/MCP_CAN_lib>`_.

The MCP2515 module is a SPI-CAN interface. The MCP_CAN library is using the SPI protocol on Arduino to program and use this module. It has been adapted here to work with the Raspberry Pi 4 GPIOs, using the SPI functions of the using the `[wiringPi] library <http://wiringpi.com/>`_.

---

One of the main difference is that we don't handle SPI Chip Select PIN. This is already done by the wiringPi library, and
all PINs for SPI are already predefined (spi channel 0 or 1).

To poll the MCP2515 module (to see if there is any data to read), the _digitalRead_ function of wiringPi is used.