#!/usr/bin/env python

from pymodbus.client.sync import ModbusTcpClient
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
import time
from enum import Enum, unique

"""
This code is an example of how to use two available vision related function in the Modbus TCP server from the client side.
It picks pawns detected in the workspace and stacks them according to their shape.
"""


# Enums for shape and color. Those enums are the one used by the modbus server to receive requests
@unique
class ColorEnum(Enum):
    ANY = -1
    BLUE = 1
    RED = 2
    GREEN = 3
    NONE = 0


@unique
class ShapeEnum(Enum):
    ANY = -1
    CIRCLE = 1
    SQUARE = 2
    TRIANGLE = 3
    NONE = 0


# Functions to convert variables for/from registers

# Positive number : 0 - 32767
# Negative number : 32768 - 65535
def number_to_raw_data(val):
    if val < 0:
        val = (1 << 15) - val
    return val


def raw_data_to_number(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val


# ---- Usefull functions

def string_to_register(string): 
    # code a string to 16 bits hex value to store in register
    builder = BinaryPayloadBuilder()
    builder.add_string(string)
    payload = builder.to_registers()
    return payload


# ----------- Modbus server related function

def back_to_observation():
    joint_real = [0.057, 0.604, -0.576, -0.078, -1.384,0.253]
    joint_real_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), joint_real))
    client.write_registers(0, joint_real_to_send)
    client.write_register(100, 1)

    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)


def open_tool():
    tool_id = get_current_tool_id()
    if tool_id == 31:
        client.write_register(513, 1)
    else:
        client.write_register(510, 1)
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.05)


def close_tool():
    tool_id = get_current_tool_id()
    if tool_id == 31:
        client.write_register(512, 1)
    else:
        client.write_register(511, 1)
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.05)


def register_workspace_name(ws_name):
    workspace_request_register = string_to_register(ws_name)
    client.write_registers(626, workspace_request_register)


def register_height_offset(height_offset):
    client.write_registers(620, int(number_to_raw_data(height_offset * 1000)))


def auto_calibration():
    print "Calibrate Robot if needed ..."
    client.write_register(311, 1)
    # Wait for end of calibration
    while client.read_input_registers(402, 1).registers[0] == 1:
        time.sleep(0.05)


def get_current_tool_id():
    return client.read_input_registers(200, count=1).registers[0]


# Functions to call Modbus Server vision functions

def detect_object(workspace_str, shape_int, color_int):
    register_workspace_name(workspace_str)

    # write requested values in registers
    client.write_registers(624, number_to_raw_data(shape_int))
    client.write_registers(625, number_to_raw_data(color_int))

    # call detect_object function
    client.write_registers(614, 1)

    # Wait for end of detect_object function
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    # - Check result : POSE
    obj_rel_pose = []
    for i in range(6):
        pose = raw_data_to_number(client.read_holding_registers(153, count=6).registers[i]) / float(1000)
        obj_rel_pose.append(pose)

    # - Check result : SHAPE AND COLOR
    result_shape_int = raw_data_to_number(client.read_holding_registers(159).registers[0])
    result_color_int = raw_data_to_number(client.read_holding_registers(160).registers[0])

    return obj_rel_pose, result_shape_int, result_color_int


def get_target_pose_from_rel(workspace_str, height_offset, x_rel, y_rel, yaw_rel):
    register_workspace_name(workspace_str)
    register_height_offset(height_offset)

    # store x_rel, y_rel and yaw_rel from detect object function
    client.write_registers(621, int(number_to_raw_data(x_rel * 1000)))
    client.write_registers(622, int(number_to_raw_data(y_rel * 1000)))
    client.write_registers(623, int(number_to_raw_data(yaw_rel * 1000)))

    # launch function
    client.write_registers(610, 1)

    # Wait for end of Get target pose from rel function
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    # - Check result : POSE
    obj_pose = []
    for i in range(6):
        pose = raw_data_to_number(client.read_holding_registers(153, count=6).registers[i]) / float(1000)
        obj_pose.append(pose)

    return obj_pose



# ----------- Main programm

if __name__ == '__main__':
    print "--- START"

    client = ModbusTcpClient('localhost', port=5020)  

    # -------- Variable definition
    workspace_name = 'workspace_modbus_vision'
    height_offset = 0.009
    release_pose_square =  [0.187, 0.184, 0.006, -0.052, 1.426, 0.621]
    release_pose_circle =  [0.282, 0.198, 0.006, -0.568, 1.46, 0.251]
    intermediate_joint = [-0.05,-0.073,-0.408, 0.018,-0.955, 0.046]
    nb_square = 0
    nb_circle = 0

    # connect to modbus server
    client.connect()
    print "Connected to modbus server"

    # launch auto calibration then go to obs. pose
    auto_calibration()
    back_to_observation()

    # updatetool
    client.write_registers(500, 1) 


    # ------------------- DETECT, PICK AND PLACE ACCORDING ON THE SHAPE ------------------------- #

    shape = ShapeEnum.ANY.value 
    color = ColorEnum.ANY.value 

    while (detect_object(workspace_name, shape, color)[1] != ShapeEnum.NONE.value):
        # while objects are detected in the workspace, grab them and stack them
        # according to their shape (circle or square).

        obj_rel_pose, shape_found, color_found = detect_object(workspace_name, shape, color)
        print 'Detected a', ColorEnum(color_found).name, ShapeEnum(shape_found).name
        
        x_rel = obj_rel_pose[0] 
        y_rel = obj_rel_pose[1]
        yaw_rel = obj_rel_pose[5]
        target_pose = get_target_pose_from_rel(workspace_name, height_offset, x_rel, y_rel, yaw_rel)

        open_tool()
        pose_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), target_pose))
        client.write_registers(10, pose_to_send)
        client.write_register(101, 1)
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)
        
        close_tool()

        # ---- Go to intermediate joints (over the workspace)
        intermediate_joint = [-0.05,-0.073,-0.408, 0.018,-0.955, 0.046]
        joints_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), intermediate_joint))
        client.write_registers(0, joints_to_send)
        client.write_register(100, 1)
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)

        # ---- Compute release pose according to the shape
        pose = [0]*6

        if shape_found == ShapeEnum.SQUARE.value:
            pose = [release_pose_square[i] if i != 2 else release_pose_square[i]+(0.015*nb_square) for i in range(6)]
            nb_square += 1
        else:
            pose = [release_pose_circle[i] if i != 2 else release_pose_circle[i]+(0.015*nb_circle) for i in range(6)]
            nb_circle += 1

        # ---- Go to release pose
        pose_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), pose))
        client.write_registers(10, pose_to_send)
        client.write_register(101, 1)
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)

        # ---- Release pawn and go back to observation 
        open_tool()
        back_to_observation()

    print 'No more objects detected in the workspace'


    # Activate learning mode and close connexion
    client.write_register(300, 1)
    client.close()
    print "Close connection to modbus server"
    print "--- END"

    