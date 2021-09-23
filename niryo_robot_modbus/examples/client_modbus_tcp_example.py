#!/usr/bin/env python

# ! You need to launch the server first !

from pymodbus.client.sync import ModbusTcpClient
import time

def number_to_raw_data(val):
    if val < 0:
        val = (1 << 15) - val
    return int(val)

if __name__ == '__main__':
    print "--- START"
    client = ModbusTcpClient('192.168.1.52', port=5020)

    client.connect()
    print "Connected to modbus server"

    print "Update tool"
    client.write_register(500, 1)
    time.sleep(2)

    print "Enable TCP"
    client.write_register(600, 1)
    time.sleep(2)

    print "Set TCP"
    tcp_rpy = [0.2, -0.1, 0.0, 0.0, 1.5, 0.0]
    tcp_rpy_to_send = list(map(lambda x: number_to_raw_data(x * 1000), tcp_rpy))
    print tcp_rpy_to_send
    client.write_registers(601, tcp_rpy_to_send)
    time.sleep(2)

    print "Disable TCP"
    client.write_register(600, 0)
    time.sleep(2)

    client.close()
    print "Close connection to modbus server"
    print "--- END"