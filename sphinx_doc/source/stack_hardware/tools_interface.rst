Niryo Robot Tools Interface Package
====================================

| This package handles Niryo's conveyor
| You can control two conveyors

Tools Interface Node
--------------------------
The ROS Node is made to :
 - Set and control tools
 - Publish tool connection state

Parameters - Tools Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``check_tool_connection_frequency``
      -  | Publish and control rate for tools connection state.
         | Default : '2.0'
   *  -  ``id_list``
      -  | List of tools id
         | Default : '[11,12,13,14,31]'
   *  -  ``motor_type_list``
      -  | List of motor tools type 
         | Default : '["xl320","xl320","xl320","xl320","xl320"]'

Published Topics - Tools Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot_hardware/tools/current_id``
      -  :std_msgs:`Int32`
      -  Current tool ID

Services - Tools Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tool Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``niryo_robot/tools/ping_and_set_dxl_tool``
      -  :ref:`tools_interface/PingDxlTool<PingDxlTool (Service)>`
      -  Scan and set for a tool plugged
   *  -  ``niryo_robot/tools/open_gripper``
      -  :ref:`tools_interface/OpenGripper<OpenGripper (Service)>`
      -  Open a gripper tool
   *  -  ``niryo_robot/tools/close_gripper``
      -  :ref:`tools_interface/CloseGripper<CloseGripper (Service)>`
      -  Close a gripper tool
   *  -  ``niryo_robot/tools/pull_air_vacuum_pump``
      -  :ref:`tools_interface/PullAirVacuumPump<PullAirVacuumPump (Service)>`
      -  Pull vacuum pump tool
   *  -  ``niryo_robot/tools/push_air_vacuum_pump``
      -  :ref:`tools_interface/PushAirVacuumPump<PushAirVacuumPump (Service)>`
      -  Push vacuum pump tool

Dependencies - Tools Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :ref:`ttl_driver <Niryo Robot Dynamixel Driver Package>`

Services & Messages files - Tools Interface
----------------------------------------------

PingDxlTool (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/tools_interface/srv/PingDxlTool.srv
   :language: rostype

OpenGripper (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/tools_interface/srv/OpenGripper.srv
   :language: rostype

CloseGripper (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/tools_interface/srv/CloseGripper.srv
   :language: rostype

PullAirVacuumPump (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/tools_interface/srv/PullAirVacuumPump.srv
   :language: rostype

PushAirVacuumPump (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/tools_interface/srv/PushAirVacuumPump.srv
   :language: rostype
