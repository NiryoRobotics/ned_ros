from .enums import *

from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ObjectColor, ObjectShape


def _object_shape_to_int(shape):
    return {ObjectShape.ANY: -1, ObjectShape.CIRCLE: 1, ObjectShape.SQUARE: 2}[shape]


def _object_color_to_int(color):
    return {ObjectColor.ANY: -1, ObjectColor.BLUE: 1, ObjectColor.RED: 2, ObjectColor.GREEN: 3}[color]
