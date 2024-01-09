from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ObjectColor, ObjectShape
"""
This module is a shared space between all the registers.
Its main goal is to allow a register to read values that has been set in other registers.
"""

linear_mode = False


def set_linear_mode(v):
    global linear_mode
    linear_mode = v


is_executing_command = False

# vision
workspace_name = 'ws_2'
height_offset = 0
shape = ObjectShape.ANY
color = ObjectColor.ANY

relative_x = 0
relative_y = 0
relative_yaw = 0
