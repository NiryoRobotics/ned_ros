int8 UPDATE=-7
int8 REBOOT=-6
int8 SHUTDOWN=-5
int8 FATAL_ERROR=-4     # Node crash
int8 MOTOR_ERROR=-3     # Electrical/overload or disconnected motor error
int8 COLLISION=-2
int8 USER_PROGRAM_ERROR=-1
int8 UNKNOWN=0
int8 BOOTING=1          # Robot is booting
int8 REBOOT_MOTOR=2
int8 CALIBRATION_NEEDED=3
int8 CALIBRATION_IN_PROGRESS=4
int8 LEARNING_MODE=5
int8 STANDBY=6          # Torque ON
int8 MOVING=7           # Moving with NiryoStudio interface or ros topics without user program
int8 RUNNING_AUTONOMOUS=8   # User program is running
int8 RUNNING_DEBUG=9    # Debug program is running
int8 PAUSE=10           # User program paused
int8 LEARNING_MODE_AUTONOMOUS=11    # User program is running + Learning mode activated
int8 LEARNING_TRAJECTORY = 12
int8 ESTOP = 13

int8 robot_status
string robot_status_str
string robot_message

int8 FATAL=-3
int8 ERROR=-2
int8 WARN=-1
int8 NONE=0

int8 logs_status
string logs_status_str
string logs_message

bool out_of_bounds
bool rpi_overheating