# Gripper
uint8 OPEN_GRIPPER = 1
uint8 CLOSE_GRIPPER = 2

# Vacuump pump
uint8 PULL_AIR_VACUUM_PUMP = 10
uint8 PUSH_AIR_VACUUM_PUMP = 11

# Tools controlled by digital I/Os
uint8 SETUP_DIGITAL_IO = 20
uint8 ACTIVATE_DIGITAL_IO = 21
uint8 DEACTIVATE_DIGITAL_IO = 22

uint8 cmd_type

# Gripper1= 11, Gripper2=12, Gripper3=13, VaccuumPump=31, Electromagnet=30
uint8 tool_id

# if gripper close
uint16 gripper_close_speed

# if gripper open
uint16 gripper_open_speed

# if vacuum pump or electromagnet grove
bool activate

# if tool is set by digital outputs (electromagnet)
# if gpio<0 get value in memory
int8 gpio
