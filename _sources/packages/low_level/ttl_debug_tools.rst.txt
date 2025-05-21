TTL Debug Tools
###############

This package is a debugging package to setup and access directly to all hardware components on the TTL bus.
It provides main functions like ping, scan device and read/write/syncRead/syncWrite operations on devices.

There are two ways to use this package: directly with the compiled binary, or via :doc:`ttl_driver` services called in dedicated scripts.

TTL debug tool binary
*********************
The compiled binary (located in *catkin_ws/install/release/lib/ttl_debug_tools/ttl_debug_tools*) directly accesses the TTL bus using :doc:`../third_parties/dynamixel_sdk` third party library.
Thus, it cannot be used if the Niryo ROS Stack is already running and you should first stop the robot stack (By typing ``sudo service niryo_robot_ros stop`` in a terminal on the robot).

This tool can be launched via:

.. code:: bash

 rosrun ttl_debug_tools ttl_debug_tools

or

.. code:: bash

 roslaunch ttl_debug_tools ttl_debug_tools

Parameters - TTL debug tools
----------------------------
    - **--help / -h:** Prints help message
    - **--baudrate / -b [Baudrate]:** Baudrate (1000000 by default)
    - **--port / -p [Port]:** Sets port
    - **--id / -i [ID]:** Device ID (-1 by default)
    - **--ids [IDs]:** Lists of devices IDs
    - **--scan:** Scans all devices on the bus
    - **--ping:** Pings specific ID
    - **--get-register [Addr]:** Gets a value from a register, parameters is: register address
    - **--get-registers [Addr]:** Gets list of values from multiple devices at a register address, parameters is: register address
    - **--get-size [Size]:** Size of data to be read with get-register or get-registers, parameters is: size of data in bytes
    - **--set-register [Addr] [Value] [Size]:** Sets a value to a register, parameters are in the order: register address / value / size (in bytes) of the data
    - **--set-registers [Addr] [Values] [Size]:** Sets values to a register on multiple devices, parameters are in the order: register address / list of values / size (in bytes) of the data
    - **--calibrate:** Calibrates all steppers on the bus. It is used in Ned2 only.

Scripts
*******
In order to use the TTL debug tools to debug an already running ROS stack, you can use 2 scripts which ensure access to the data on the TTL bus via 2 services:

- read_custom_dxl_value.py : uses the ``/niryo_robot/ttl_driver/read_custom_value`` service to read values from the TTL bus.
- send_custom_dxl_value.py : uses the ``/niryo_robot/ttl_driver/write_custom_value`` to write values to the TTL bus.

Niryo robot - Send DXL custom value
-----------------------------------
It uses a ttl_driver service to send data to a register of a device on the TTL bus when the ROS stack is running.
This script can be launched via:

.. code:: bash

 rosrun ttl_debug_tools send_custom_dxl_value.py

Parameters - Send custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--id [ID]:** Device ID
    - **--address [Addr]:** Registers address to modify
    - **--value [Value]:** Value to store at the register address given
    - **--size [Size]:** Size in bytes of the data to write

Niryo robot - Read DXL custom value
-----------------------------------
It uses a service to read data from a given register of a device on the TTL bus when the ROS stack is running.
This script can be launched via:

.. code:: bash

 rosrun ttl_debug_tools read_custom_dxl_value.py

Parameters - Read custom value
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - **--id [ID]:** Device ID
    - **--address [Addr]:** Register address to modify
    - **--size [Size]:** Size in bytes of the data to read

