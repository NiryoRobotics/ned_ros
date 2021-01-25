int32 MOVE_ONLY = 1
int32 TOOL_ONLY = 2

int32 cmd_type

niryo_robot_commander/ArmMoveCommand arm_cmd

niryo_robot_tools/ToolCommand tool_cmd

# In the future, allow a tool command to be launched at the same time as an Arm command
# 3 choices : arm only, arm + tool, tool only
