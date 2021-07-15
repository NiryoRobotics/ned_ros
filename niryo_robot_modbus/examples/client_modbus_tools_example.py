#!/usr/bin/env python

# ! You need to launch the server first !

from pymodbus.client.sync import ModbusTcpClient
import time

if __name__ == '__main__':
    print "--- START"
    client = ModbusTcpClient('localhost', port=5020)

    client.connect()
    print "Connected to modbus server"

    # print "Pull vaccuum pump"
    # client.write_register(512, 1)
    # time.sleep(2)

    # print "Push vaccuum pump"
    # client.write_register(513, 1)
    # time.sleep(2)

    print "Update tool"
    client.write_register(500, 1)
    time.sleep(2)

    print "Set speed open gripper"
    client.write_register(401, 500)
    time.sleep(2)

    print "Set speed close gripper"
    client.write_register(402, 500)
    time.sleep(2)

    print "Open Gripper"
    client.write_register(510, 1)
    time.sleep(2)

    print "Close Gripper"
    client.write_register(511, 1)
    time.sleep(2)

    client.close()
    print "Close connection to modbus server"
    print "--- END"