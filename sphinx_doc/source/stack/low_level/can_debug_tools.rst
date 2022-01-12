CAN Debug Tools
====================================

This package offers scripts to debug with Hardware and setup CAN devices.
It provides some main functions like setup the CAN bus and dump data on bus.

Niryo robot - CAN debug tools
------------------------------------
It provides service to dump data on CAN bus.
This script can be launched via:  ::

 rosrun can_debug_tools can_debug_tools

Parameters - CAN debug tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--help / -h:** Print help message
    - **--baudrate / -b [Baudrate]:** Baudrate (1000000 by default)
    - **--channel / -c [Channel]:** Set channel SPI (0 by default)
    - **--gpio / -g:** GPIO Interrupt for CAN (25 by default)
    - **--freq / -f:** frequency of control loop to check data (100Hz by default)
    - **--dump:** run dump service to dump and show all data found on bus

When you dump data on CAN bus, the result is a table including:
    - Number of data's package 
    - Status of package
    - Control byte
    - Data in 8 bytes


.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/can_debug_tools