TTL Debug Tools
====================================

This package offers scripts to change ping/scan DXL motors and changes register values of these motors.


Niryo robot - Ttl debug tools
------------------------------------
This script can be launched via:  ::

 rosrun ttl_debug_tools ttl_debug_tools

Parameters - Ttl debug tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--help / -h:** Prints help message
    - **--baudrate / -b [Number]:** Baudrates (1000000 by default)
    - **--port / -p [Number]:** Sets port
    - **--id / -i [Number]:** Dxl motor ID (0 by default)
    - **--scan:** Scans all Dxl motors on the bus
    - **--ping:** Pings specific ID
    - **--set-register [Number] [Number] [Number]:** Sets a value to a register, parameters are in the order: register address / value / size (in bytes) of the value

Services, Topics and Messages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. todo:: pas de services et topics et tout ? 


Scripts
------------------------------------

Niryo robot - Send DXL custom value
------------------------------------
This script can be launched via:  ::

 rosrun ttl_debug_tools send_custom_dxl_value.py

Parameters - Send custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--id [Number]:** Motor ID
    - **--address [Number]:** Registers address to modify
    - **--value [Number]:** Value to store at the register address given
    - **--size [Number]:** Size in bytes of the value given

Niryo robot - Read DXL custom value
------------------------------------
This script can be launched via:  ::

 rosrun ttl_debug_tools read_custom_dxl_value.py

Parameters - Read custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--id [Number]:** Motor ID
    - **--address [Number]:** Registers address to modify
    - **--value [Number]:** Value to store at the register address given
    - **--size [Number]:** Size in bytes of the value given
