#!/usr/bin/env python

# ! You need to launch the server first !

from pymodbus.client.sync import ModbusTcpClient

from input_register_data_block import *
from discrete_input_data_block import *
from coil_data_block import *
from holding_register_data_block import *

import time


def number_to_raw_data(val):
    if val < 0:
        val = (1 << 15) - val
    return val


def raw_data_to_number(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val


def wait_end_of_execution(client):
    # Wait for end of Move command
    while client.read_holding_registers(HR_IS_EXECUTING_CMD, count=1).registers[0] == 1:
        time.sleep(0.01)


def test_coil_data(client):
    print("---- COIL DATA BLOCK TEST BEGINS ----")

    digital_IO_modes = client.read_discrete_inputs(DI_DIGITAL_IO_MODE, 8)
    print("BEFORE MODIFICATION DIGITAL IO MODES: {}".format(digital_IO_modes.bits))

    digital_IO_states = client.read_discrete_inputs(DI_DIGITAL_IO_STATE, 8)
    print("BEFORE MODIFICATION DIGITAL IO STATES: {}".format(digital_IO_states.bits))

    # Set digital IO mode - output
    client.write_coil(0, False)
    client.write_coil(3, False)

    # Set digital IO state
    client.write_coil(100, False)
    client.write_coil(103, True)

    time.sleep(0.2)

    digital_IO_modes = client.read_discrete_inputs(DI_DIGITAL_IO_MODE, 8)
    print("AFTER MODIFICATION DIGITAL IO MODES: {}".format(digital_IO_modes.bits))

    digital_IO_states = client.read_discrete_inputs(DI_DIGITAL_IO_STATE, 8)
    print("AFTER MODIFICATION DIGITAL IO STATES: {}".format(digital_IO_states.bits))

    print("---- COIL DATA BLOCK TEST ENDS ----")


def test_discrete_input(client):
    print("---- DISCRETE INPUT TEST BEGINS ----")

    digital_IO_modes = client.read_discrete_inputs(DI_DIGITAL_IO_MODE, 8)
    print("DIGITAL IO MODES: {}".format(digital_IO_modes.bits))

    digital_IO_states = client.read_discrete_inputs(DI_DIGITAL_IO_STATE, 8)
    print("DIGITAL IO STATES: {}".format(digital_IO_states.bits))

    print("---- DISCRETE INPUT TEST ENDS ----")


def test_input_register(client):
    print("---- INPUT REGISTERS TEST BEGINS ----")

    joints_registers = client.read_input_registers(IR_JOINTS, 6)
    joints = [raw_data_to_number(item) / 1000.00 for item in joints_registers.registers[:]]
    print("JOINTS: {}".format(joints))

    pose_registers = client.read_input_registers(IR_POSITION_X, 6)
    pose_list = [raw_data_to_number(item) / 1000.00 for item in pose_registers.registers[:]]
    print("POSE: {}".format(pose_list))

    selected_tool_registers = client.read_input_registers(IR_SELECTED_TOOL_ID, 1)
    print("SELECTED TOOL: {}".format(selected_tool_registers.registers[0]))

    learning_mode_registers = client.read_input_registers(IR_LEARNING_MODE, 1)
    print("LEARNING MODE: {}".format(learning_mode_registers.registers[0]))

    hardware_status_registers = client.read_input_registers(IR_MOTORS_CONNECTION_UP, 10)
    print("HW STATUS / MOTORS CONNECTION UP: {}".format(hardware_status_registers.registers[0]))
    print("HW STATUS / CALIBRATION NEEDED: {}".format(hardware_status_registers.registers[1]))
    print("HW STATUS / CALIBRATION IN PROGRESS: {}".format(hardware_status_registers.registers[2]))
    print("HW STATUS / RPI TEMPERATURE: {}".format(hardware_status_registers.registers[3]))
    print("HW STATUS / RPI AVAILABLE SPACE: {}".format(hardware_status_registers.registers[4]))
    print("HW STATUS / RPI ROS LOG SIZE: {}".format(hardware_status_registers.registers[5]))
    print("HW STATUS / RPI IMAGE VERSION: {}".format(hardware_status_registers.registers[6:9]))
    print("HW STATUS / HARDWARE_VERSION: {}".format(hardware_status_registers.registers[9]))

    conveyor_1_registers = client.read_input_registers(IR_CONVEYOR_1_CONNECTION_STATE, 4)
    print("CONVEYOR 1 STATE: {}".format(conveyor_1_registers.registers[0]))
    print("CONVEYOR 1 CONTROL STATUS: {}".format(conveyor_1_registers.registers[1]))
    print("CONVEYOR 1 SPEED: {}".format(conveyor_1_registers.registers[2]))
    print("CONVEYOR 1 DIRECTION: {}".format(conveyor_1_registers.registers[3]))

    conveyor_2_registers = client.read_input_registers(IR_CONVEYOR_2_CONNECTION_STATE, 4)
    print("CONVEYOR 1 STATE: {}".format(conveyor_2_registers.registers[0]))
    print("CONVEYOR 1 CONTROL STATUS: {}".format(conveyor_2_registers.registers[1]))
    print("CONVEYOR 1 SPEED: {}".format(conveyor_2_registers.registers[2]))
    print("CONVEYOR 1 DIRECTION: {}".format(conveyor_2_registers.registers[3]))

    print("---- INPUT REGISTERS TEST ENDS ----")


def test_holding_register(client):
    print("---- HOLDING REGISTER TEST BEGINS ----")

    print("Force recalibration")
    client.write_register(HR_NEW_CALIBRATION_REQUEST, 1)

    print("Calibrate Robot if needed")
    client.write_register(HR_START_AUTO_CALIBRATION, 1)
    time.sleep(1)

    wait_end_of_execution(client)
    print("Calibration ended")

    print("Send a Joint Move command to the robot")
    joints_to_send = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    client.write_registers(HR_JOINTS, joints_to_send)
    client.write_register(HR_MOVE_JOINTS_COMMAND, 1)
    time.sleep(1)
    wait_end_of_execution(client)

    print("Joint Move command is finished")

    print("Send a move pose command to the robot")
    pose_to_send = [0.304, 0.231, 0.322, 0.0, 0.178, 0.648]
    pose_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), pose_to_send))

    client.write_register(HR_POSITION_X, pose_to_send[0])
    client.write_register(HR_POSITION_Y, pose_to_send[1])
    client.write_register(HR_POSITION_Z, pose_to_send[2])
    client.write_register(HR_ORIENTATION_X, pose_to_send[3])
    client.write_register(HR_ORIENTATION_Y, pose_to_send[4])
    client.write_register(HR_ORIENTATION_Z, pose_to_send[5])
    client.write_register(HR_MOVE_POSE_COMMAND, 1)
    time.sleep(1)
    wait_end_of_execution(client)

    print("Move pose command is finished")

    print("Testing tool stuff begins")

    client.write_register(HR_OPEN_GRIPPER, 1)
    wait_end_of_execution(client)
    client.write_register(HR_CLOSE_GRIPPER, 1)
    wait_end_of_execution(client)
    client.write_register(HR_PULL_AIR_VACUUM_PUMP, 1)
    wait_end_of_execution(client)
    client.write_register(HR_PUSH_AIR_VACUUM_PUMP, 1)
    wait_end_of_execution(client)

    print("Testing tool stuff end")

    learning_mode = client.read_holding_registers(HR_LEARNING_MODE, 1)
    print("Learning mode current state: {}".format(learning_mode.registers[0]))

    print("Settings learning mode to {}".format(not learning_mode.registers[0]))
    client.write_register(HR_LEARNING_MODE, not learning_mode.registers[0])
    time.sleep(1)
    print("Settings learning mode to previous state {}".format(not learning_mode.registers[0]))
    client.write_register(HR_LEARNING_MODE, learning_mode.registers[0])

    print("---- CONVEYOR HOLDING REGISTER TEST BEGINS ----")
    client.write_register(HR_PING_AND_SET_CONVEYOR, 1)
    wait_end_of_execution(client)
    conveyor_id_scanned = client.read_holding_registers(HR_LAST_ROBOT_CMD_DATA_RESULT, 1).registers[0]
    print("New conveyor ID scanned: {}".format(conveyor_id_scanned))

    if conveyor_id_scanned is 0:
        print("Starting conveyor at full speed in forward...")
        client.write_register(HR_CONTROL_CONVEYOR_SPEED, 100)
        client.write_register(HR_CONTROL_CONVEYOR_DIRECTION, 1)
        client.write_register(HR_CONTROL_CONVEYOR_ID, conveyor_id_scanned)
        client.write_register(HR_CONTROL_CONVEYOR, 1)
        wait_end_of_execution(client)
        print("Conveyor should be running now")
        time.sleep(2)
        print("Changing direction...")
        client.write_register(HR_CONTROL_CONVEYOR_DIRECTION, 1)
        wait_end_of_execution(client)
        time.sleep(2)
        print("Stopping ...")
        client.write_register(HR_CONTROL_CONVEYOR_ID, conveyor_id_scanned)
        client.write_register(HR_STOP_CONVEYOR_WITH_ID, 1)
        wait_end_of_execution(client)
        print("Conveyor should be stopped now")
        print("Removing conveyor now...")
        client.write_register(HR_CONTROL_CONVEYOR_ID, conveyor_id_scanned)
        client.write_register(HR_REMOVE_CONVEYOR_WITH_ID, 1)
        wait_end_of_execution(client)
        print("Conveyor should be removed now")
    else:
        print("No conveyor scanned :'(")
    print("---- CONVEYOR HOLDING REGISTER TEST ENDS ----")
    print("---- HOLDING REGISTER TEST ENDS ----")


if __name__ == '__main__':
    need_test_coil_data = True
    need_test_discrete_input = True
    need_test_input_register = True
    need_test_holding_register = True

    modbus_ip_address = '127.0.0.1'
    modbus_port = 5020

    print "--- START"
    modbus_tcp_client = ModbusTcpClient(modbus_ip_address, port=modbus_port)

    modbus_tcp_client.connect()
    print "Connected to modbus server"

    if need_test_holding_register:
        test_holding_register(modbus_tcp_client)

    if need_test_coil_data:
        test_coil_data(modbus_tcp_client)

    if need_test_discrete_input:
        test_discrete_input(modbus_tcp_client)

    if need_test_input_register:
        test_input_register(modbus_tcp_client)

    modbus_tcp_client.close()
    print "Close connection to modbus server"
    print "--- END"
