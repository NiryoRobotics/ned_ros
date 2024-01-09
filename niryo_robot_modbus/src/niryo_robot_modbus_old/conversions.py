from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ObjectColor, ObjectShape

object_color_to_int = {ObjectColor.ANY: -1, ObjectColor.BLUE: 1, ObjectColor.RED: 2, ObjectColor.GREEN: 3}
int_to_object_color = {v: k for k, v in object_color_to_int.items()}

object_shape_to_int = {ObjectShape.ANY: -1, ObjectShape.CIRCLE: 1, ObjectShape.SQUARE: 2}
int_to_object_shape = {v: k for k, v in object_shape_to_int.items()}
