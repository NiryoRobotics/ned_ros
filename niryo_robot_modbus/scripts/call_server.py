import argparse
import builtins
import math

from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.pdu import ModbusExceptions


def read_modbus(server_ip, registry_type, registry_address, count=1, data_type=None):
    client = ModbusTcpClient(server_ip, 5020)

    if registry_type.lower() == 'coil':
        data_type = bool
        response = client.read_coils(registry_address, count)
    elif registry_type.lower() == 'discrete_input':
        data_type = bool
        response = client.read_discrete_inputs(registry_address, count)
    elif registry_type.lower() == 'holding_register':
        response = client.read_holding_registers(registry_address, count)
    elif registry_type.lower() == 'input_register':
        response = client.read_input_registers(registry_address, count)
    else:
        print("Invalid registry type. Supported types: coil, discrete_input, holding_register, input_register")
        return

    if response.isError():
        if not hasattr(response, 'exception_code'):
            print(f'read at address {registry_address} failed: {response}')
            return
        print(f'read at address {registry_address} failed: {ModbusExceptions.decode(response.exception_code)}')
        return

    if data_type:
        decoded_response = decode_response(response, data_type)
        print(decoded_response)
    else:
        print(response.registers)


def decode_response(response, data_type):
    if data_type == bool:
        return response.bits
    elif data_type == int:
        return response.registers
    decoder = BinaryPayloadDecoder.fromRegisters(response.registers)

    if data_type == float:
        n_float = math.ceil(len(response.registers) / 2)
        return [decoder.decode_32bit_float() for _ in range(n_float)]
    elif data_type == str:
        return decoder.decode_string(200)
    else:
        print("Invalid data type. Supported types: int, float, str, bool")
        return None


def to_bool(value):
    return value in ['True', 'true', True, 1, 1.0]


def write_modbus(server_ip, registry_type, registry_address, values, data_type=None):
    client = ModbusTcpClient(server_ip, 5020)

    if registry_type.lower() == 'coil':
        values = [to_bool(value) for value in values]
        response = client.write_coils(registry_address, values)
    elif registry_type.lower() == 'holding_register':
        if data_type == int:
            payload = [int(value) for value in values]
        else:
            builder = BinaryPayloadBuilder()
            for value in values:
                if data_type == float:
                    builder.add_32bit_float(float(value))
                elif data_type == str:
                    builder.add_string(value)
                else:
                    print("Invalid data type. Supported types: int, float, str")
                    return
            payload = builder.to_registers()
        response = client.write_registers(registry_address, payload)

        if response.isError():
            if not hasattr(response, 'exception_code'):
                print(f'write at address {registry_address} failed: {response}')
                return
            print(f'write at address {registry_address} failed: {ModbusExceptions.decode(response.exception_code)}')
            return

    else:
        print("Invalid registry type. Supported types: coil, holding_register")
        return

    print(response)


def main():
    parser = argparse.ArgumentParser(description="Modbus Client Script")
    parser.add_argument("server_ip", type=str, help="Modbus server IP address")
    parser.add_argument("registry_type",
                        type=str,
                        choices=["coil", "discrete_input", "holding_register", "input_register"],
                        help="Modbus registry type")
    parser.add_argument("registry_address", type=int, help="Modbus registry address")
    parser.add_argument("--count", type=int, default=1, help="Number of registers to read (default is 1)")
    parser.add_argument("--values", nargs='+', help="Values to write to the registers")
    parser.add_argument("--data_type",
                        type=lambda type_name: getattr(builtins, type_name),
                        choices=[int, float, str, bool],
                        help="Data type for decoding the response")

    args = parser.parse_args()

    if args.values:
        write_modbus(args.server_ip, args.registry_type, args.registry_address, args.values, args.data_type)
    else:
        read_modbus(args.server_ip, args.registry_type, args.registry_address, args.count, args.data_type)


if __name__ == "__main__":
    main()
