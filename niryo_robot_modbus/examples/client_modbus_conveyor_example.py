#!/usr/bin/env python

# ! You need to launch the server first !

from pymodbus.client import ModbusTcpClient
import time


# Positive number : 0 - 32767
# Negative number : 32768 - 65535
def number_to_raw_data(val):
    if val < 0:
        val = (1 << 15) - val
    return val


def raw_data_to_number(val):
    if (val >> 15) == 1:
        val = -(val & 0x7FFF)
    return val


if __name__ == '__main__':
    print("--- START")
    client = ModbusTcpClient('192.168.1.122', port=5020)

    client.connect()
    print("Connected to modbus server")

    print("Ping and set the conveyor")
    client.write_register(520, 1)
    time.sleep(2)

    print("set the direction to backward")
    client.write_register(523, number_to_raw_data(-1))
    time.sleep(1)

    print("set the speed to 50%")
    client.write_register(524, 50)
    time.sleep(1)

    print("start the conveyor plugged")
    client.write_register(522, 1)
    time.sleep(5)

    print("stop the conveyor")
    client.write_register(526, 1)
    time.sleep(1)

    print("set the direction to forward")
    client.write_register(523, 1)
    time.sleep(1)

    print("set the speed to 100%")
    client.write_register(524, 100)
    time.sleep(1)

    print("start the conveyor plugged")
    client.write_register(522, 1)
    time.sleep(5)

    print("stop the conveyor plugged")
    client.write_register(526, 1)
    time.sleep(1)

    print("Remove the conveyor")
    client.write_register(521, 1)
    time.sleep(1)

    client.close()
    print("Close connection to modbus server")
    print("--- END")
