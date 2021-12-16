TTL Debug Tools
====================================

This package offers scripts to change ping/scan DXL motors and changes register values of these motors.


Niryo robot - Ttl debug tools
------------------------------------
This script can be launched via:  ::

 rosrun ttl_debug_tools ttl_debug_tools

Parameters - Ttl debug tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--help / -h:** Print help message
    - **--baudrate / -b [Number]:** Baudrate (1000000 by default)
    - **--port / -p [Number]:** Set port
    - **--id / -i [Number]:** Dxl motor ID (0 by default)
    - **--scan:** Scan all Dxl motors on the bus
    - **--ping:** Ping specific ID
    - **--set-register [Number] [Number] [Number]:** Set a value to a register, parameters are in the order: register address / value / size (in bytes) of the value

Services, Topics and Messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



Scripts
------------------------------------

Niryo robot - Send DXL custom value
------------------------------------
This script can be launched via:  ::

 rosrun ttl_debug_tools send_custom_dxl_value.py

Parameters - Send custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--id [Number]:** Motor ID
    - **--address [Number]:** Register address to modify
    - **--value [Number]:** Value to store at the register address given
    - **--size [Number]:** Size in bytes of the value given

Niryo robot - Read DXL custom value
------------------------------------
This script can be launched via:  ::

 rosrun ttl_debug_tools read_custom_dxl_value.py

Parameters - Read custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--id [Number]:** Motor ID
    - **--address [Number]:** Register address to modify
    - **--value [Number]:** Value to store at the register address given
    - **--size [Number]:** Size in bytes of the value given
