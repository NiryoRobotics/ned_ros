Use the Modbus TCP server
====================================

:todo: vu avec Etienne, supprimer la partie Modbus d'ici et mettre en place une redirection vers doc Modbus.

Ned is permanently running a Modbus TCP Server that enables Ned to communicate with a PLC, or another computer in the same network.

Connection - Modbus TCP server
------------------------------------
The Modbus TCP server is running on port 5020 by default.

Description -  Modbus TCP server
------------------------------------
It has been built on top of the `pymodbus library <https://pymodbus.readthedocs.io/en/latest/index.html>`_.

All 4 Modbus datastores are implemented:
    - Coil.
    - Discrete Input.
    - Holding Register.
    - Input Register.

Each datastore has a different set of functionalities. Note that each datastore contains a completely different set of data.

Address tables start at 0.

Coil
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Each address contains a 1bit value.

READ/WRITE (the stored values correspond to the last given command, not the current robot state).

Accepted Modbus functions:
    - 0x01: READ_COILS
    - 0x05: WRITE_SINGLE_COIL

This datastore can be used to set Digital I/O mode and state.

Digital I/O numbers used for Modbus:
    - 0/100: 1A
    - 1/101: 1B
    - 2/102: 1C
    - 3/103: 2A
    - 4/104: 2B
    - 5/105: 2C

.. list-table::
   :header-rows: 1
   :widths: auto
   :stub-columns: 0

   *  -  Address
      -  Description
   *  -  0-5
      -  Digital I/O mode (Input = 1, Output = 0)
   *  -  100-105
      -  Digital I/O state (High = 1, Low = 0)
   *  -  200-299
      -  Can be used to store your own variables

Discrete input
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Each address contains a 1bit value.

READ-ONLY

Accepted Modbus functions:
    - 0x02: READ_DISCRETE_INPUTS

Digital I/O numbers used for Modbus:
    - 0/100: 1A
    - 1/101: 1B
    - 2/102: 1C
    - 3/103: 2A
    - 4/104: 2B
    - 5/105: 2C

.. list-table::
   :header-rows: 1
   :widths: auto
   :stub-columns: 0

   *  -  Address
      -  Description
   *  -  0-5
      -  Digital I/O mode (Input = 1, Output = 0)
   *  -  100-105
      -  Digital I/O state (High = 1, Low = 0)

This datastore can be used to read Digital I/O mode and state of the robot. See :ref:`source/modbus_tcp_server:Coil` above for digital I/O number mapping. 

Holding register
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Each address contains a 16bits value.

READ/WRITE (the stored values correspond to the last given command, not the current robot state)

Accepted Modbus functions:
    - 0x03: READ_HOLDING_REGISTERS
    - 0x06: WRITE_SINGLE_REGISTER

.. list-table::
   :header-rows: 1
   :widths: auto
   :stub-columns: 0

   *  -  Address
      -  Description
   *  -  0-5
      -  Joints (mrad)
   *  -  10-12
      -  Position x,y,z (mm)
   *  -  13-15
      -  Orientation roll, pitch, yaw (mrad)
   *  -  100
      -  Sends Joint Move command with stored joints
   *  -  101
      -  Sends Pose Move command with stored position and orientation
   *  -  102
      -  Sends Pose Linear Move command with stored position and orientation
   *  -  110
      -  Stops current command execution
   *  -  150
      -  Is executing command flag
   *  -  151
      -  Last command result (status of the last command)
   *  -  152
      -  Contains data retrieved from last cmd (depends of the cmd)
   *  -  153 - 158
      -  Vision - Target pose result
   *  -  159
      -  Vision - Shape of the object found (-1: ANY, 1: CIRCLE, 2: SQUARE, 3: TRIANGLE, 0: NONE)
   *  -  160
      -  Vision - Color of the object found (-1: ANY, 1: BLUE, 2: RED, 3: GREEN, 0: NONE)
   *  -  200-299
      -  Can be used to store your own variables
   *  -  300
      -  Learning Mode (On = 1, Off = 0)
   *  -  301
      -  Joystick Enabled (On = 1, Off = 0)
   *  -  310
      -  Requests new calibration
   *  -  311
      -  Starts auto calibration
   *  -  312
      -  Starts manual calibration
   *  -  401
      -  Gripper open speed (100-1000)
   *  -  402
      -  Gripper close speed (100-1000)
   *  -  500
      -  Updates the tool id according to the gripper plugged (gripper 1: 11, gripper 2: 12, gripper 3: 13, vaccum pump: 31)
   *  -  501
      -  Stores the tool id 
   *  -  510
      -  Opens gripper previously updated
   *  -  511
      -  Closes gripper with given id
   *  -  512 
      -  Pulls air vacuum pump from given id
   *  -  513 
      -  Pushes air vacuum pump from given id
   *  -  520 
      -  Enables a Conveyor Belt newly connected [on success: store its ID at 152]
   *  -  521 
      -  Detaches / disables Conveyor Belt with the Conveyor Belt ID given at 525
   *  -  522 
      -  Control Conveyor Belt with the Conveyor Belt ID given at 525
   *  -  523 [related to 522]
      -  Conveyor Belt direction (backward = -1 , forward = 1)
   *  -  524 [related to 522]
      -  Conveyor Belt speed (0-100)(%)
   *  -  525 [related to 520/521/522/526]
      -  Stores the Conveyor Belt ID for all related command
   *  -  526 
      -  Stops Conveyor Belt with the Conveyor Belt ID given at 525
   *  -  600
      -  TCP - Enables or disables the TCP function (Tool Center Point). 
   *  -  601
      -  Activates the TCP function (Tool Center Point) and defines the transformation between the tool_link frame and the TCP frame.
   *  -  610
      -  Vision - Gets target pose from relative pose, with stored relative pose and height_offset
   *  -  611
      -  Vision - Gets target pose from camera, with stored workspace name, height offset, shape and color
   *  -  612
      -  Vision - Vision pick, with stored workspace name, height offset, shape and color
   *  -  613
      -  Vision - Moves to object, with stored workspace name, height offset, shape and color
   *  -  614
      -  Vision - Detects object, with stored workspace name, shape and color
   *  -  620
      -  Vision - Stores workspace's height offset
   *  -  621
      -  Vision - Stores relative pose x_rel
   *  -  622
      -  Vision - Stores relative pose y_rel
   *  -  623
      -  Vision - Stores relative pose yaw_rel
   *  -  624
      -  Vision - Stores requested shape (-1: ANY, 1: CIRCLE, 2: SQUARE, 3: TRIANGLE)
   *  -  625
      -  Vision - Stores requested color (-1: ANY, 1: BLUE, 2: RED, 3: GREEN)
   *  -  626 - max 641
      -  Vision - Stores workspace's name, as a string encoded in 16 bits hex (see examples on how to store a workspace name from a client)

Input Register
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each address contains a 16bits value.

READ-ONLY

Accepted Modbus functions:
    - 0x04: READ_INPUT_REGISTERS

.. list-table::
   :header-rows: 1
   :widths: auto
   :stub-columns: 0

   *  -  Address
      -  Description
   *  -  0-5
      -  Joints (mrad)
   *  -  10-12
      -  Position x,y,z (mm)
   *  -  13-15
      -  Orientation roll, pitch, yaw (mrad)
   *  -  200
      -  Selected tool ID (0 for no tool)
   *  -  300
      -  Learning Mode activated
   *  -  400
      -  Motors connection up (Ok = 1, Not ok = 0)
   *  -  401
      -  Calibration needed flag
   *  -  402
      -  Calibration in progress flag
   *  -  403
      -  Raspberry Pi temperature
   *  -  404
      -  Raspberry Pi available disk size
   *  -  405
      -  Raspberry Pi ROS log size
   *  -  406
      -  RPI software version n.1
   *  -  407
      -  RPI software version n.2
   *  -  408
      -  RPI software version n.3
   *  -  409
      -  Hardware version (1 or 2)
   *  -  530
      -  Conveyor 1 connection state (Connected = 1 , Not connected = 0)
   *  -  531
      -  Conveyor 1 control status ( On = 0, Off = 1)
   *  -  532
      -  Conveyor 1 Speed (0-100 (%))
   *  -  533
      -  Conveyor 1 direction (Backward = -1, Forward = 1)
   *  -  540
      -  Conveyor 2 connection state (Connected = 1 , Not connected = 0)
   *  -  541
      -  Conveyor 2 control status ( On = 0, Off = 1)
   *  -  542
      -  Conveyor 2 Speed (0-100 (%))
   *  -  543
      -  Conveyor 2 direction (Backward = -1, Forward = 1)

Dependencies - Modbus TCP Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- `pymodbus library <https://pymodbus.readthedocs.io/en/latest/index.html>`_
- :doc:`ros/niryo_robot_msgs`
- :msgs_index:`std_msgs`
