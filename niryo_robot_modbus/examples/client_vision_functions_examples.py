#!/usr/bin/env python

from pymodbus.client.sync import ModbusTcpClient
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
import time
from enum import Enum, unique

"""
This code is an example of how to use each available vision related function in the Modbus TCP server from the client side.
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


def print_result(func_name=None, args=None, result=None):
    print func_name, 'called with arguments :',
    for arg in args:
        print arg,
    print ' | Returned :',
    for res in result:
        print res,
    print '\n'
    


# ----------- Modbus server related function

def pick_and_place_from_pose(pose):
    if pose != [0] * 6:
        open_tool()

        print '--pick and place from pose :', pose
        pose_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), pose))
        client.write_registers(10, pose_to_send)
        client.write_register(101, 1)

        # Wait for end of Move command
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)

        # - GRAB OBJECT
        close_tool()

        # ---- Go to release pose
        joints = [0.866, -0.24, -0.511, 0.249, -0.568, -0.016]
        joints_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), joints))

        client.write_registers(0, joints_to_send)
        client.write_register(100, 1)

        # Wait for end of Move command
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)

        open_tool()

    else:
        print 'Pick and place failed - target pose is null'
        return


def back_to_observation():
    # To change
    # joint_real = [0.057, 0.604, -0.576, -0.078, -1.384,0.253] 
    joint_simu = [0, -0.092, 0, 0, -1.744, 0]

    joint_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), joint_simu))
    client.write_registers(0, joint_to_send)
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


def get_target_pose_from_cam(workspace_str, height_offset, shape_int, color_int):
    register_workspace_name(workspace_str)
    register_height_offset(height_offset)

    client.write_registers(624, number_to_raw_data(shape_int))
    client.write_registers(625, number_to_raw_data(color_int))

    # launch get_target_pose_from_cam function
    client.write_registers(611, 1)

    # Wait for end of function
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    # - Check result : POSE
    obj_pose = []
    for i in range(6):
        pose = raw_data_to_number(client.read_holding_registers(153, count=6).registers[i]) / float(1000)
        obj_pose.append(pose)

    return obj_pose


def move_to_object(workspace_str, height_offset, shape_int, color_int):
    register_workspace_name(workspace_str)
    register_height_offset(height_offset)

    client.write_registers(624, number_to_raw_data(shape_int))
    client.write_registers(625, number_to_raw_data(color_int))

    # launch move to object function
    client.write_registers(613, 1)

    # Wait for end of function
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    # - Check result : SHAPE AND COLOR
    result_shape_int = raw_data_to_number(client.read_holding_registers(159).registers[0])
    result_color_int = raw_data_to_number(client.read_holding_registers(160).registers[0])

    return result_shape_int, result_color_int


def vision_pick(workspace_str, height_offset, shape_int, color_int):
    register_workspace_name(workspace_str)
    register_height_offset(height_offset)

    client.write_registers(624, number_to_raw_data(shape_int))
    client.write_registers(625, number_to_raw_data(color_int))

    # launch vision pick function
    client.write_registers(612, 1)

    # Wait for end of function
    while client.read_holding_registers(150, count=1).registers[0] == 1:
        time.sleep(0.01)

    # - Check result : SHAPE AND COLOR
    result_shape_int = raw_data_to_number(client.read_holding_registers(159).registers[0])
    result_color_int = raw_data_to_number(client.read_holding_registers(160).registers[0])

    return result_shape_int, result_color_int


# ----------- Main programm

if __name__ == '__main__':
    print "--- START"

    client = ModbusTcpClient('localhost', port=5020)  

    # -------- Variable definition
    # To change
    workspace_name = 'workspace_modbus_vision'
    height_offset = 0.0

    # connect to modbus server
    client.connect()
    print "Connected to modbus server"

    # launch auto calibration then go to obs. pose
    auto_calibration()
    back_to_observation()

    # update and open tool
    client.write_registers(500, 1) 
    open_tool()


    # ------------------- MOVEMENTS AND VISION ------------------------- #

    # ----- DETECT OBJECT
    print '\n'
    print '------ DETECT OBJECT : get rel. pose of any blue pawn'

    shape = ShapeEnum.ANY.value 
    color = ColorEnum.BLUE.value  
    obj_rel_pose, shape_found, color_found = detect_object(workspace_name, shape, color)

    print_result('detect_object', [ShapeEnum(shape).name, ColorEnum(color).name],
                 [ShapeEnum(shape_found).name, ColorEnum(color_found).name, 'at pose', obj_rel_pose])

    back_to_observation()


    # ------ GET TARGET POSE FROM REL
    print '\n'
    print '------ GET TARGET POSE FROM REL : get target pose from blue pawn rel. pose, and pick&place it'
    
    # Get object detected previously
    x_rel = obj_rel_pose[0] 
    y_rel = obj_rel_pose[1]
    yaw_rel = obj_rel_pose[5]

    target_pose = get_target_pose_from_rel(workspace_name, height_offset, x_rel, y_rel, yaw_rel)
    print_result('get_target_pose_from_rel', [x_rel, y_rel, yaw_rel], ['target pose', target_pose])

    # pick and place the object, then back to obs.
    pick_and_place_from_pose(target_pose)
    back_to_observation()


    # ----- GET TARGET POSE FROM CAM
    print '\n'
    print '------ GET TARGET POSE FROM CAM : get target pose of any red pawn and pick&place it'

    shape = ShapeEnum.ANY.value  
    color = ColorEnum.RED.value 
    obj_pose = get_target_pose_from_cam(workspace_name, height_offset, shape, color)
    print_result('get_target_pose_from_cam', [ShapeEnum(shape).name, ColorEnum(color).name],
                 ['target pose', obj_pose])

    # pick and place the object, then back to obs.
    pick_and_place_from_pose(obj_pose)
    back_to_observation()


    # ----- MOVE TO OBJECT function
    print '\n'
    print '------ MOVE TO OBJECT - move over any green pawn'

    shape = ShapeEnum.ANY.value  
    color = ColorEnum.GREEN.value  
    shape_found, color_found = move_to_object(workspace_name, height_offset, shape, color)
    print_result('move_to_object', [ShapeEnum(shape).name, ColorEnum(color).name],
                 [ShapeEnum(shape_found).name, ColorEnum(color_found).name])

    back_to_observation()


    # ----- Vision pick function
    print '\n'
    print '------ VISION PICK - pick any red pawn, lift it and release it'

    shape = ShapeEnum.ANY.value  
    color = ColorEnum.RED.value  
    shape_picked, color_picked = vision_pick(workspace_name, height_offset, shape, color)
    print_result('vision_pick', [ShapeEnum(shape).name, ColorEnum(color).name],
                 ['picked ', ShapeEnum(shape_picked).name, ColorEnum(color_picked).name])

    open_tool()
    back_to_observation()

    # Activate learning mode and close connexion
    client.write_register(300, 1)
    client.close()
    print "Close connection to modbus server"
    print "--- END"
