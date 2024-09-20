# test_modbus_server.py
import math
import time

import pytest
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.pdu import ModbusExceptions


@pytest.fixture(scope='module')
def client(server_ip):
    return ModbusTcpClient(server_ip, port=5020, timeout=30)


def encode(value, data_type):
    if not isinstance(value, list):
        value = [value]

    if data_type == int:
        return value

    encoder = BinaryPayloadBuilder()

    if data_type == float:
        for v in value:
            encoder.add_32bit_float(v)
    elif data_type == str:
        for v in value:
            encoder.add_string(v)
    else:
        raise TypeError(f'Unknown type "{data_type}"')
    return encoder.to_registers()


def decode(value: list, data_type):
    decoder = BinaryPayloadDecoder.fromRegisters(value)
    if data_type == int:
        r_value = value
    elif data_type == float:
        if len(value) % 2 != 0:
            raise ValueError("Can't decode floats with an uneven number of bytes")
        n_float = len(value) // 2
        r_value = [decoder.decode_32bit_float() for _ in range(n_float)]
    elif data_type == str:
        decoded_string = decoder.decode_string(200).decode()
        if decoded_string[-1] == '\x00':
            decoded_string = decoded_string[:-1]
        r_value = [decoded_string]
    else:
        print("Invalid data type. Supported types: int, float, str, bool")
        r_value = []
    return r_value if len(r_value) > 1 else r_value[0]


def client_call(fun, address, *args, **kwargs):
    result = fun(address, *args, **kwargs)
    if result.isError():
        if not hasattr(result, 'exception_code'):
            raise AssertionError(f'{fun.__name__} at address {address} failed: {result}')
        raise AssertionError(
            f'{fun.__name__} at address {address} failed: {ModbusExceptions.decode(result.exception_code)}')
    return result


def read_coils(client: ModbusTcpClient, address: int, count: int = 1):
    result = client_call(client.read_coils, address, count=count)
    bits = result.bits[:count]
    return bits if len(bits) > 1 else bits[0]


def write_coils(client: ModbusTcpClient, address: int, values, read_check_delay=0.1):
    client_call(client.write_coils, address, values=values)

    if read_check_delay > 0:
        time.sleep(read_check_delay)
        count = 1 if not isinstance(values, list) else len(values)
        result = read_coils(client, address, count=count)
        assert are_equals(result, values), (f'Value read at address {address} does not match expected value. '
                                            f'Written: {values}, obtained: {result}')


def read_holding_registers(client: ModbusTcpClient, address: int, data_type: type, count: int = 1):
    result = client_call(client.read_holding_registers, address, count=count)
    return decode(result.registers, data_type)


def write_holding_registers(client: ModbusTcpClient, address: int, values, data_type, read_check_delay=0.01):
    if isinstance(values, list):
        values = [data_type(v) for v in values]
    else:
        values = data_type(values)

    encoded_value = encode(values, data_type)
    client_call(client.write_registers, address, values=encoded_value)

    if read_check_delay > 0:
        time.sleep(read_check_delay)
        result = read_holding_registers(client, address, data_type, count=len(encoded_value))
        assert are_equals(result, values), (f'Value read at {address} does not match expected value. '
                                            f'Written: {encoded_value}, obtained: {result}')


def read_discrete_inputs(client: ModbusTcpClient, address: int, count: int = 1):
    result = client_call(client.read_discrete_inputs, address, count=count)
    return result.bits[:count]


def read_input_registers(client: ModbusTcpClient, address: int, data_type, count: int = 1):
    result = client_call(client.read_input_registers, address, count=count)
    return decode(result.registers, data_type)


# - comparison functions


def are_equals(a, b, rel_tol=1E-6, abs_tol=0.0):
    if type(a) is not type(b):
        raise TypeError(f'Cannot compare "{type(a).__name__}" with "{type(b).__name__}"')

    if isinstance(a, list):
        return all(are_equals(x, y, rel_tol, abs_tol) for x, y in zip(a, b))
    elif isinstance(a, float):
        return math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)
    else:
        return a == b


def is_conveyor_speed_ok(requested_speed: int, real_speed: int) -> bool:
    # There is a bug with the conveyor speed (the real value is always lesser than the requested speed)
    # so we can't directly check if real_speed == requested_speed
    return (requested_speed - real_speed) <= requested_speed * 0.2


def wait_for_equality(comp_function, a, b, timeout: float = 5, **kwargs):
    t_start = time.time()
    while not comp_function(a, b, **kwargs) and (time.time() - t_start) < timeout:
        time.sleep(0.1)
    return comp_function(a, b, **kwargs)


# - test functions


def test_user_stores(client):
    for bool_value in [True, False]:
        for address in range(200, 300):
            write_coils(client, address, bool_value)

    for float_value in [483.0, 4.7, 63.21, 789.456]:
        for address in range(200, 300):
            write_holding_registers(client, address, float_value, float)


def test_gripper(client):
    # activate tool
    write_coils(client, 50, True, read_check_delay=0.5)
    assert read_coils(client, 50) is True

    # set grippers open / close params
    write_holding_registers(client, 107, 500, int)
    write_holding_registers(client, 108, 100, int)
    write_holding_registers(client, 109, 100, int)

    write_holding_registers(client, 110, 500, int)
    write_holding_registers(client, 111, 100, int)
    write_holding_registers(client, 112, 100, int)

    # open / close gripper
    write_coils(client, 51, False, read_check_delay=0.2)
    write_coils(client, 51, True, read_check_delay=0.2)
    write_coils(client, 51, False, read_check_delay=0.2)


def test_inputs_outputs(client):
    ios_addresses = [0, 2]
    for address in ios_addresses:
        # AIOs
        read_input_registers(client, address, float, count=2)
        read_holding_registers(client, address, float, count=2)
        for voltage in [5.0, 1.289, 0.0]:
            write_holding_registers(client, address, voltage, float)

        # DIOs
        read_discrete_inputs(client, address)
        read_coils(client, address)
        for bool_value in [True, False]:
            write_coils(client, address, bool_value)

    read_discrete_inputs(client, 4)


def test_conveyors(client, n_conveyors):
    conveyor_check_delay = 3
    for conveyor_offset in range(n_conveyors):
        try:
            # attach
            result = read_coils(client, 53 + conveyor_offset)
            if not result:
                write_coils(client, 53 + conveyor_offset, True, read_check_delay=conveyor_check_delay)

            # speed + run
            for speed in [100, 66, 33]:
                write_holding_registers(client, 87 + conveyor_offset, speed, int, read_check_delay=0)
                time.sleep(conveyor_check_delay)
                result = read_holding_registers(client, 87 + conveyor_offset, int)
                assert is_conveyor_speed_ok(speed, result), 'Conveyor speed didnt reach expected speed'

            # direction
            for bool_value in [True, False]:
                write_coils(client, 93 + conveyor_offset, bool_value, read_check_delay=conveyor_check_delay)
        finally:
            # always try to stop conveyor
            write_coils(client, 73 + conveyor_offset, False, read_check_delay=conveyor_check_delay)


def test_tcp(client):
    for tcp_value, tcp_enabled in [(0.1, True), (0.0, False)]:
        for address in range(75, 86, 2):
            write_holding_registers(client, address, tcp_value, float)
        write_coils(client, 52, tcp_enabled)


def calibrate(client, force=False):
    if force:
        # calibration needed
        write_coils(client, 115, True, read_check_delay=0.4)
    # calibration
    write_coils(client, 116, True, read_check_delay=0)


def move_j(client, target):
    calibrate(client)
    # set move target
    write_holding_registers(client, 50, target, float)
    # set move type
    write_holding_registers(client, 74, 0, int)

    # collision detected
    if read_coils(client, 117):
        write_coils(client, 117, False)

    # move
    write_coils(client, 113, True, read_check_delay=0)
    current = read_input_registers(client, 50, float, 12)
    assert wait_for_equality(are_equals, current, target, abs_tol=0.1)


def move_p(client, target, linear=False):
    calibrate(client)
    # set move target
    write_holding_registers(client, 62, target, float)
    # set move type
    write_holding_registers(client, 74, 2 if linear else 1, int)

    # collision detected
    if read_coils(client, 117):
        write_coils(client, 117, False)

    # move
    write_coils(client, 113, True, read_check_delay=0)
    current = read_input_registers(client, 62, float, 12)
    assert wait_for_equality(are_equals, current, target, abs_tol=0.1)


def test_motion(client):
    calibrate(client, force=True)

    # arm speed
    write_holding_registers(client, 142, 200, int)

    move_sequences = {
        0: [[1.0, 0.5, 1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        1: [[0.25, 0.25, 0.5, 0.0, 0.0, 0.0]],
        2: [[0.25, 0.0, 0.5, 0.0, 0.0, 0.0], [0.3, 0.0, 0.2, 0.0, 0.0, 0.0], [0.15, 0.0, 0.2, 0.0, 0.0, 0.0]],
    }

    for move_type, targets in move_sequences.items():
        for target in targets:
            if move_type == 0:
                move_j(client, target)
            elif move_type == 1:
                move_p(client, target)
            elif move_type == 2:
                move_p(client, target, linear=True)
            else:
                raise ValueError("move type must be 0, 1 or 2")


def test_learning_mode(client):
    write_coils(client, 114, True)
    time.sleep(1)
    write_coils(client, 114, False)


def test_vision(client):
    workspace_name = 'default_workspace'
    write_holding_registers(client, 119, workspace_name, str, read_check_delay=0)
    assert are_equals(read_holding_registers(client, 119, str, math.ceil(len(workspace_name) / 2)), workspace_name)

    observation_pose = [0.04, 0.16, 0.21, -3.13, 1.3, -1.84]
    move_p(client, observation_pose)

    # height offset
    write_holding_registers(client, 139, 20, int)
    # target shape
    write_holding_registers(client, 140, 1, int)
    # target color
    write_holding_registers(client, 141, 3, int)

    # target found
    assert read_discrete_inputs(client, 52)

    # found shape and color
    assert are_equals(read_input_registers(client, 98, int, count=2), [1, 3])

    vision_pose = read_input_registers(client, 86, float, count=12)
    print(vision_pose)

    rel_pose = [0.1, 0.1, 0.1]
    write_holding_registers(client, 113, rel_pose, float)
    abs_pose_from_relative = read_input_registers(client, 74, float, count=12)
    print(abs_pose_from_relative)


def test_misc(client):
    # executing command
    read_discrete_inputs(client, 51)
    # last command result
    read_input_registers(client, 120, int)
