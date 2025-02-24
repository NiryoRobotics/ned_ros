# Gripper
int8 OPEN_GRIPPER = 1
int8 CLOSE_GRIPPER = 2

# Vacuump pump
int8 PULL_AIR_VACUUM_PUMP = 10
int8 PUSH_AIR_VACUUM_PUMP = 11

# Tools controlled by digital I/Os
int8 SETUP_DIGITAL_IO = 20
int8 ACTIVATE_DIGITAL_IO = 21
int8 DEACTIVATE_DIGITAL_IO = 22

uint8 cmd_type

# Gripper1= 11, Gripper2=12, Gripper3=13, VacuumPump=31, VacuumPumpV2=32, Electromagnet=30
int8 tool_id

# if gripper Ned1/One
uint16 speed

# if gripper Ned2
uint8 max_torque_percentage
uint8 hold_torque_percentage

# if vacuum pump or electromagnet grove
bool activate

# if tool is set by digital outputs (electromagnet)
string gpio
