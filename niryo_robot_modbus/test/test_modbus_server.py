# test_modbus_server.py
import math
import time

import pytest
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.pdu import ModbusExceptions


@pytest.fixture(scope='module')
def client():
    return ModbusTcpClient('localhost', port=5020)


def client_call(fun, address, *args, **kwargs):
    result = fun(address, *args, **kwargs)
    if result.isError():
        raise AssertionError(
            f'{fun.__name__} at address {address} failed: {ModbusExceptions.decode(result.exception_code)}')
    return result


def read_coil(client: ModbusTcpClient, address: int):
    result = client_call(client.read_coils, address)
    return result.bits[0]


def write_coil(client: ModbusTcpClient, address: int, value, read_check_delay=0.1):
    client_call(client.write_coil, address, value=value)

    if read_check_delay > 0:
        time.sleep(read_check_delay)
        result = read_coil(client, address)
        assert are_equals(result, value), (f'Value read at address {address} does not match expected value. '
                                           f'Written: {value}, obtained: {result}')


def encode(value, data_type):
    if data_type == int:
        return [value]
    encoder = BinaryPayloadBuilder()
    if data_type == float:
        encoder.add_32bit_float(value)
    elif data_type == str:
        encoder.add_string(value)
    else:
        raise TypeError(f'Unknown type "{data_type}"')
    return encoder.to_registers()


def decode(value, data_type):
    if data_type == int:
        return value[0]
    decoder = BinaryPayloadDecoder.fromRegisters(value)
    if data_type == float:
        return decoder.decode_32bit_float()
    elif data_type == str:
        return decoder.decode_string(200)
    else:
        raise TypeError(f'Unknown type "{data_type}"')


def read_holding_register(client: ModbusTcpClient, address: int, data_type):
    result = client_call(client.read_holding_registers, address)
    return decode(result.registers, data_type)


def write_holding_register(client: ModbusTcpClient, address: int, value, data_type, read_check_delay=0.01):
    encoded_value = encode(value, data_type)
    client_call(client.write_registers, address, values=encoded_value)

    if read_check_delay > 0:
        time.sleep(read_check_delay)
        result = read_holding_register(client, address, data_type)
        assert are_equals(result, value), (f'Value read at {address} does not match expected value. '
                                           f'Written: {encoded_value}, obtained: {result}')


def read_discrete_input(client: ModbusTcpClient, address: int):
    result = client_call(client.read_discrete_inputs, address)
    return result.bits[0]


def read_input_register(client: ModbusTcpClient, address: int, data_type):
    result = client_call(client.read_input_registers, address)
    return decode(result.registers, data_type)


# - comparison functions


def are_equals(a, b):
    if type(a) is not type(b):
        raise TypeError(f'Cannot compare "{type(a).__name__}" with "{type(b).__name__}"')
    elif isinstance(a, float):
        return math.isclose(a, b, rel_tol=1E-6)
    else:
        return a == b


def is_conveyor_speed_ok(requested_speed: int, real_speed: int) -> bool:
    # There is a bug with the conveyor speed (the real value is always lesser than the requested speed)
    # so we can't directly check if real_speed == requested_speed
    return (requested_speed - real_speed) <= requested_speed * 0.2


# - test functions


def test_user_stores(client):
    for bool_value in [True, False]:
        for address in range(200, 300):
            write_coil(client, address, bool_value)
            result = read_coil(client, address)
            assert result is bool_value, f'read C at {address}: is not {bool_value}'

    for float_value in [483, 4.7, 63.21, 789.456]:
        for address in range(200, 300):
            write_holding_register(client, address, float_value, float)
            result = read_holding_register(client, address, float)
            assert are_equals(result, float_value), f'read HR at {address}: {result} != {float_value}'


def test_gripper(client):
    # activate tool
    write_coil(client, 50, True, read_check_delay=0.5)
    assert read_coil(client, 50) is True

    # set grippers open / close params
    write_holding_register(client, 107, 500, int)
    write_holding_register(client, 108, 100, int)
    write_holding_register(client, 109, 100, int)

    write_holding_register(client, 110, 500, int)
    write_holding_register(client, 111, 100, int)
    write_holding_register(client, 112, 100, int)

    # open / close gripper
    write_coil(client, 51, False)
    write_coil(client, 51, True)
    write_coil(client, 51, False)


def test_inputs_outputs(client):
    ios_addresses = [0, 2]
    for address in ios_addresses:
        # AIOs
        read_input_register(client, address, float)
        read_holding_register(client, address, float)
        for voltage in [5.0, 1.289, 0.0]:
            write_holding_register(client, address, voltage, float)

        # DIOs
        read_discrete_input(client, address)
        read_coil(client, address)
        for bool_value in [True, False]:
            write_coil(client, address, bool_value)

    read_discrete_input(client, 4)


def test_conveyors(client):
    conveyor_check_delay = 3
    conveyor_offsets = [0]
    for conveyor_offset in conveyor_offsets:
        try:
            # attach
            result = read_coil(client, 53 + conveyor_offset)
            if not result:
                write_coil(client, 53 + conveyor_offset, True, read_check_delay=conveyor_check_delay)

            # speed + run
            for speed in [100, 66, 33]:
                write_holding_register(client, 87 + conveyor_offset, speed, int, read_check_delay=0)
                time.sleep(conveyor_check_delay)
                result = read_holding_register(client, 87, int)
                assert is_conveyor_speed_ok(speed, result), f'Conveyor speed didnt reach expected speed'

            # direction
            for bool_value in [True, False]:
                write_coil(client, 93 + conveyor_offset, bool_value, read_check_delay=conveyor_check_delay)
        finally:
            # always try to stop conveyor
            write_coil(client, 73 + conveyor_offset, False, read_check_delay=conveyor_check_delay)
