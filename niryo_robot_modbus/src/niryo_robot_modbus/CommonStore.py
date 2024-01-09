from niryo_robot_python_ros_wrapper import ObjectShape, ObjectColor

from .util import MoveType


class CommonStore:

    is_executing_command = False

    # move
    move_type = MoveType.MOVE_JOINT
    joint_target = [0] * 6
    pose_target = [0] * 6

    # vision
    workspace_name = ''
    height_offset = 0
    shape = ObjectShape.ANY
    color = ObjectColor.ANY

    relative_x = 0
    relative_y = 0
    relative_yaw = 0

    coil_user_store = {key: False for key in range(100)}
