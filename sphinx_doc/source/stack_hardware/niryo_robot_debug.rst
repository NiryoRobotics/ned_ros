Dxl_debug_tools
====================================

This package offers scripts to change ping/scan DXL motors and changes register values of these motors.

Niryo robot - Send DXL custom value
------------------------------------
This script can be launched via:  ::

 rosrun niryo_robot_debug send_custom_dxl_value.py

Parameters - Send DXL custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--type [Number]:** Motor type (3 for XL-320, 2 for XL-430)
    - **--id [Number]:** Motor ID
    - **--address [Number]:** Register address to modify
    - **--value [Number]:** Value to store at the register address given
    - **--size [Number]:** Size in bytes of the value given

Niryo robot - Dxl debug tools
------------------------------------
This script can be launched via:  ::

 rosrun niryo_robot_debug dxl_debug_tools

Parameters - Dxl debug tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--help / -h:** Print help message
    - **--baudrate / -b [Number]:** Baudrate (1000000 by default)
    - **--port / -p [Number]:** Set port
    - **--id / -i [Number]:** Dxl motor ID (0 by default)
    - **--scan:** Scan all Dxl motors on the bus
    - **--ping:** Ping specific ID
    - **--set-register [Number] [Number] [Number]:** Set a value to a register, parameters are in the order: register address / value / size (in bytes) of the value
