Modbus Examples
-------------------------------

Examples of Modbus python lib can be found here :modbus_examples:`Python Modbus examples<>`.
In the examples folder, you can find several example scripts that control Ned. These scripts are commented to help you understand every step.

Client Modbus Test
############################################################
Calls several functions on the IO of Ned.

Client Move Command
############################################################
This script shows the calibration and Ned's moves.

Client Modbus Conveyor Example
############################################################
This script shows how to activate the Conveyor Belt through the Modbus Python API, set a direction, a speed, and start and stop the device. 

Client Modbus Vision Example
############################################################
This script shows how to use the vision pick method from a Modbus Client, through the Modbus Python API. Ned picks
a red object seen in its workspace and releases it on its left. 
Note that we use the **string_to_register** method to convert a string into an object storable in registers. ::

    #!/usr/bin/env python

    from pymodbus.client.sync import ModbusTcpClient
    from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
    import time
    from enum import Enum, unique

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

    def string_to_register(string): 
        # code a string to 16 bits hex value to store in register
        builder = BinaryPayloadBuilder()
        builder.add_string(string)
        payload = builder.to_registers()
        return payload

    # ----------- Modbus server related function

    def back_to_observation():
        # To change
        # joint_real = [0.057, 0.604, -0.576, -0.078, -1.384,0.253] 
        joint_simu = [0, -0.092, 0, 0, -1.744, 0]

        joint_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), joint_simu))
        client.write_registers(0, joint_to_send)
        client.write_register(100, 1)

        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)

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

    def open_tool():
        tool_id = get_current_tool_id()
        if tool_id == 31:
            client.write_register(513, 1)
        else:
            client.write_register(510, 1)
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.05)


    # Function to call Modbus Server vision pick function
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
        workspace_name = 'gazebo_1'
        height_offset = 0.0

        # connect to modbus server
        client.connect()
        print "Connected to modbus server"

        # launch auto calibration then go to obs. pose
        auto_calibration()
        back_to_observation()

        # update tool
        client.write_registers(500, 1) 

        print 'VISION PICK - pick a red pawn, lift it and release it'
        shape = ShapeEnum.ANY.value  
        color = ColorEnum.RED.value  
        shape_picked, color_picked = vision_pick(workspace_name, height_offset, shape, color)

        # ---- Go to release pose
        joints = [0.866, -0.24, -0.511, 0.249, -0.568, -0.016]
        joints_to_send = list(map(lambda j: int(number_to_raw_data(j * 1000)), joints))

        client.write_registers(0, joints_to_send)
        client.write_register(100, 1)

        # Wait for end of Move command
        while client.read_holding_registers(150, count=1).registers[0] == 1:
            time.sleep(0.01)

        open_tool()

        back_to_observation()

        # Activate learning mode and close connexion
        client.write_register(300, 1)
        client.close()
        print "Close connection to modbus server"
        print "--- END"

