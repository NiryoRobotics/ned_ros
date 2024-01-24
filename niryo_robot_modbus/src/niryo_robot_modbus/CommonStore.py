from .util import MoveType


class CommonStore:

    is_executing_command = False
    last_command_result = 0

    # move
    move_type = MoveType.MOVE_JOINT
    joint_target = []
    pose_target = []

    # gripper
    gripper_open_speed = 500
    gripper_open_max_torque = 100
    gripper_open_hold_torque = 20
    gripper_close_speed = 500
    gripper_close_max_torque = 100
    gripper_close_hold_torque = 50

    # vision
    workspace_name = ''
    height_offset = 0
    target_shape = 0
    target_color = 0

    relative_x = 0
    relative_y = 0
    relative_yaw = 0
