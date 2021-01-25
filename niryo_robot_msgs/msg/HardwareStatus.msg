std_msgs/Header header

# Raspberry Pi board
int32 rpi_temperature 

# Robot version : 1 Ned (only one until needed modification)
int32 hardware_version

# Motors
bool connection_up
string error_message
bool calibration_needed
bool calibration_in_progress

string[] motor_names
string[] motor_types

int32[] temperatures
float64[] voltages
int32[] hardware_errors
string[] hardware_errors_message
