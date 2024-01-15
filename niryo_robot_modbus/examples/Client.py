from __future__ import annotations

import time

from pymodbus.client import ModbusTcpClient

from typing import Callable, Union

from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.pdu import ModbusResponse, ModbusExceptions

SupportedType = Union[bool, float, int, str]


def client_call(fn: Callable, address: int, *args, **kwargs) -> ModbusResponse:
    response = fn(address, *args, **kwargs)
    if response.isError():
        exception_name = ModbusExceptions.decode(response.exception_code)
        if exception_name == 'IllegalAddress':
            raise ValueError(f'Illegal address "{address}"')
        else:
            raise ValueError(f'Can\'t get register at address "{address}": {str(response)}')
    else:
        return response


def float_payload(*args):
    builder = BinaryPayloadBuilder()
    for f in args:
        builder.add_32bit_float(f)
    return builder.to_registers()


def string_payload(string):
    builder = BinaryPayloadBuilder()
    builder.add_string(string)
    return builder.to_registers()


def move_j(*args):
    client_call(client.write_registers, 50, float_payload(*args))
    client_call(client.write_registers, 74, 0)
    time.sleep(1)
    client_call(client.write_coil, 113, True)


def move_p(*args):
    client_call(client.write_registers, 62, float_payload(*args))
    client_call(client.write_registers, 74, 1)
    time.sleep(1)
    client_call(client.write_coil, 113, True)


def calibrate(force=False):
    if force:
        client_call(client.write_coil, 115, 1)
    client_call(client.write_coil, 116, 1)


if __name__ == '__main__':
    client = ModbusTcpClient('192.168.1.50', 5020)
    calibrate(force=True)
    # move_j(0, 0, 0, 0, 0, 0)
    # time.sleep(2)
    # move_p(0.2, 0, 0.2, 0, 0, 0)
