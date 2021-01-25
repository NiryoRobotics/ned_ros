uint8 tool_id
uint8 cmd_type

# if gripper close
uint16 gripper_close_speed

# if gripper open
uint16 gripper_open_speed

# if vacuum pump or electromagnet grove
bool activate

# if tool is set by digital outputs (electromagnet)
# if gpio<0 get value in memory
int8 gpio
