import argparse
import builtins

from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder


def read_modbus(server_ip, registry_type, registry_address, count=1, data_type=None):
    client = ModbusTcpClient(server_ip, 5020)

    if registry_type.lower() == 'coil':
        data_type = bool
        response = client.read_coils(registry_address, count)
    elif registry_type.lower() == 'discrete_input':
        response = client.read_discrete_inputs(registry_address, count)
    elif registry_type.lower() == 'holding_register':
        response = client.read_holding_registers(registry_address, count)
    elif registry_type.lower() == 'input_register':
        response = client.read_input_registers(registry_address, count)
    else:
        print("Invalid registry type. Supported types: coil, discrete_input, holding_register, input_register")
        return

    if data_type:
        decoded_response = decode_response(response, data_type)
        print(decoded_response)
    else:
        print(response.registers)


def decode_response(response, data_type):
    if data_type == bool:
        return response.bits[0]
    elif data_type == int:
        return response.registers[0]
    decoder = BinaryPayloadDecoder.fromRegisters(response.registers)

    if data_type == float:
        return decoder.decode_32bit_float()
    elif data_type == str:
        return decoder.decode_string()
    else:
        print("Invalid data type. Supported types: int, float, str, bool")
        return None


def write_modbus(server_ip, registry_type, registry_address, values, data_type=None):
    client = ModbusTcpClient(server_ip, 5020)

    if registry_type.lower() == 'coil':
        response = client.write_coils(registry_address, values)
    elif registry_type.lower() == 'holding_register':

        if data_type == int:
            payload = values
        else:
            builder = BinaryPayloadBuilder()
            for value in values:
                if data_type == float:
                    builder.add_32bit_float(value)
                elif data_type == str:
                    builder.add_string(value)
                else:
                    print("Invalid data type. Supported types: int, float, str")
                    return
            payload = builder.to_registers()
        response = client.write_registers(registry_address, payload)
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
    parser.add_argument("--values", type=int, nargs='+', help="Values to write to the registers")
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
