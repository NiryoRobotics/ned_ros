Use the Modbus TCP server
====================================

.. image:: ../../images/modbus_logo.jpg
         :alt: Modbus logo
         :width: 400px
         :align: center

In this document, we will focus on the Modbus/TCP server.

Ned is permanently running a Modbus TCP Server that enables Ned to communicate with a PLC, or another computer in the same network.

The Modbus/TCP server is running on **port 5020** by default.
It has been built on top of the `pymodbus <https://pymodbus.readthedocs.io/en/latest/index.html>`_ library.
This enables you to make Ned communicates with a PLC, or another computer on the same network.

Introduction
------------

All 4 Modbus datastores are implemented: :ref:`Coils<source/modbus/api_documentation:Coils>`, :ref:`Discrete inputs<source/modbus/api_documentation:Discrete inputs>`, :ref:`Holding registers<source/modbus/api_documentation:Holding registers>`, :ref:`Input registers<source/modbus/api_documentation:Input registers>`.
Each datastore has a different set of functionalities. Note that each datastore contains a completely different set of data.

Discrete Input and Input register are **READ-ONLY** tables. Those have been used to keep the robot state.

Coil and Holding Register are **READ/WRITE** tables. Those have been used to give user commands to the robot.
Hence, those 2 tables do not contain the robot state, but the last given command.

Address tables start at 0.


Coils
-------------------------------

Each address contains a 1bit value.

**READ/WRITE** (the stored values correspond to the last given command, not the current robot state)

Accepted Modbus functions:

- 0x01: READ_COILS
- 0x05: WRITE_SINGLE_COIL

This datastore can be used to set Digital I/O mode and state. Digital I/O numbers used for Modbus:

.. list-table:: Digital IO addresses offset table
   :header-rows: 1
   :widths: auto
   :align: center

   *  - Address offset
      - Niryo One / Ned digital IO
      - Ned2 digital IO
   *  - 0
      - 1A
      - DI1
   *  - 1
      - 1B
      - DI2
   *  - 2
      - 1C
      - DI3
   *  - 3
      - 2A
      - DI4
   *  - 4
      - 2B
      - DI5
   *  - 5
      - 2C
      - D01
   *  - 6
      - SW1
      - D02
   *  - 7
      - SW2
      - D03
   *  - 8
      -
      - D04


.. list-table::
   :header-rows: 1
   :widths: auto
   :align: center

   *  - Address
      - Description

   *  - 0-8
      - Digital I/O mode (Input = 1, Output = 0)

   *  - 100-108
      - Digital I/O state (High = 1, Low = 0)

   *  - 200-299
      - Can be used to store your own variables


Discrete inputs
-------------------------------

Each address contains a 1bit value.

**READ-ONLY**

Accepted Modbus functions:

- 0x02: READ_DISCRETE_INPUTS

This datastore can be used to read Digital I/O mode and state. See the :ref:`source/modbus/api_documentation:Coils` section above for digital I/O number mapping.

.. list-table::
   :header-rows: 1
   :widths: auto
   :align: center

   *  - Address
      - Description

   *  - 0-8
      - Digital I/O mode (Input = 1, Output = 0)

   *  - 100-108
      - Digital I/O state (High = 1, Low = 0)


Holding registers
-------------------------------

Each address contains a 16bit value.

**READ/WRITE** (the stored values correspond to the last given command, not the current robot state)

Accepted Modbus functions:

- 0x03: READ_HOLDING_REGISTERS
- 0x06: WRITE_SINGLE_REGISTER

.. list-table::
   :header-rows: 1
   :widths: auto
   :align: center

   *  - Address
      - Description

   *  - 0-5
      - Joints (mrad)

   *  - 10-12
      - Position x,y,z (mm)
      
   *  - 13-15
      - Orientation roll, pitch, yaw (mrad)
      
   *  - 100
      - Sends Joint Move command with stored joints
      
   *  - 101
      - Sends Pose Move command with stored position and orientation

   *  - 102
      - Sends Linear Pose Move command with stored position and orientation
  
   *  - 110
      - Stops current command execution
      
   *  - 150
      - Is executing command flag
      
   *  - 151
      - Last command result*

   *  - 152
      - Last command data result (if not vision related)

   *  - 153 - 158
      - Vision - Target pose result

   *  - 159
      - Vision - Shape of the object found (-1: ANY, 1: CIRCLE, 2: SQUARE, 3: TRIANGLE, 0: NONE)

   *  - 160
      - Vision - Color of the object found (-1: ANY, 1: BLUE, 2: RED, 3: GREEN, 0: NONE)
  
   *  - 200-299
      - Can be used to store your own variables
      
   *  - 300
      - Learning Mode (On = 1, Off = 0)
      
   *  - 301
      - Joystick Enabled (On = 1, Off = 0)
      
   *  - 310
      - Requests new calibration
      
   *  - 311
      - Starts auto calibration
      
   *  - 312
      - Starts manual calibration
      
   *  - 401
      - Gripper open speed (100-1000)
      
   *  - 402
      - Gripper close speed (100-1000)
      
   *  - 500
      - Updates the tool id according to the gripper plugged (gripper 1: 11, gripper 2: 12, gripper 3: 13, vaccum pump: 31)

   *  - 501
      - Stores the tool id 
      
   *  - 510
      - Opens gripper previously updated
      
   *  - 511
      - Closes gripper previously updated
      
   *  - 512
      - Pulls air vacuum pump with id 31
      
   *  - 513
      - Pushes air vacuum pump with id 31
      
   *  - 520
      - Updates the conveyor id and enable it
      
   *  - 521
      - Detaches or disables the conveyor previously enabled and updated
      
   *  - 522
      - Starts the conveyor previously enabled and updated
      
   *  - 523
      - Sets the conveyor direction (backward = number_to_raw_data(-1), forward = 1)
      
   *  - 524
      - Sets the conveyor speed (0-100)(%)
      
   *  - 525
      - Stores the conveyor id
      
   *  - 526
      - Stops conveyor previously enabled and updated

   *  - 600
      - TCP - Enables or disables the TCP function (Tool Center Point). 

   *  - 601
      - Activates the TCP function (Tool Center Point) and defines the transformation between the tool_link frame and the TCP frame.

   *  - 610
      - Vision - Gets target pose from relative pose, with stored relative pose and height_offset

   *  - 611
      - Vision - Gets target pose from camera, with stored workspace name, height offset, shape and color

   *  - 612
      - Vision - Vision pick, with stored workspace name, height offset, shape and color
   
   *  - 613
      - Vision - Moves to object, with stored workspace name, height offset, shape and color
   
   *  - 614
      - Vision - Detects object, with stored workspace name, shape and color
   
   *  - 620
      - Vision - Stores workspace's height offset
   
   *  - 621
      - Vision - Stores relative pose x_rel
   
   *  - 622
      - Vision - Stores relative pose y_rel
   
   *  - 623
      - Vision - Stores relative pose yaw_rel
   
   *  - 624
      - Vision - Stores requested shape (-1: ANY, 1: CIRCLE, 2: SQUARE, 3: TRIANGLE)
   
   *  - 625
      - Vision - Stores requested color (-1: ANY, 1: BLUE, 2: RED, 3: GREEN)
   
   *  - 626 - max 641
      - Vision - Stores workspace's name, as a string encoded in 16 bits hex (see examples on how to store a workspace name from a client)

   *  - 650
      - Set Analog IO - Arg: [:ref:`Analog IO number<Analog IO addresses offset table>`, voltage 0V- 5000mV]

'*' The "Last command result" gives you more information about the last executed command:

- 0: no result yet
- 1: success
- 2: command was rejected (invalid params, ...)
- 3: command was aborted
- 4: command was canceled
- 5: command had an unexpected error
- 6: command timeout
- 7: internal error


Input registers
-------------------------------

Each address contains a 16bit value.

**READ-ONLY**.

Accepted Modbus functions:

- 0x04: READ_INPUT_REGISTERS

.. list-table::
   :header-rows: 1
   :widths: auto
   :align: center

   *  - Address
      - Description

   *  - 0-5
      - Joints (mrad)

   *  - 10-12
      - Position x,y,z (mm)
      
   *  - 13-15
      - Orientation roll, pitch, yaw (mrad)
      
   *  - 200
      - Selected tool ID (0 for no tool)
      
   *  - 300
      - Learning Mode activated
      
   *  - 400
      - Motors connection up (Ok = 1, Not ok = 0)
      
   *  - 401
      - Calibration needed flag
      
   *  - 402
      - Calibration in progress flag
      
   *  - 403
      - Raspberry Pi temperature
      
   *  - 404
      - Raspberry Pi available disk size
      
   *  - 405
      - Raspberry Pi ROS log size
      
   *  - 406
      - Ned RPI image version n.1
      
   *  - 407
      - Ned RPI image version n.2
      
   *  - 408
      - Ned RPI image version n.3
      
   *  - 409
      - Hardware version (1 or 2)
      
   *  - 530
      - Conveyor 1 connection state (Connected = 1 , Not connected = 0)
      
   *  - 531
      - Conveyor 1 control status ( On = 0, Off = 1)
      
   *  - 532
      - Conveyor 1 Speed (0-100 (%))
      
   *  - 533
      - Conveyor 1 direction (Backward = -1, Forward = 1)
      
   *  - 540
      - Conveyor 2 connection state (Connected = 1 , Not connected = 0)
      
   *  - 541
      - Conveyor 2 control status ( On = 0, Off = 1)
      
   *  - 542
      - Conveyor 2 Speed (0-100 (%))
      
   *  - 543
      - Conveyor 2 direction (Backward = -1, Forward = 1)

   *  - 600 - 604
      - Analog IO mode

   *  - 610 - 614
      - Analog IO state in mV


.. _Analog IO addresses offset table:

.. list-table:: Analog IO addresses offset table
   :header-rows: 1
   :widths: auto
   :align: center

   *  - Address offset
      - Niryo One / Ned analog IO
      - Ned2 analog IO
   *  - 0
      - /
      - AI1
   *  - 1
      - /
      - AI2
   *  - 2
      - /
      - AO1
   *  - 3
      - /
      - AO2



Dependencies - Modbus TCP Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- `pymodbus library <https://pymodbus.readthedocs.io/en/latest/index.html>`_
- :doc:`../stack/high_level/niryo_robot_msgs`
- :msgs_index:`std_msgs`
