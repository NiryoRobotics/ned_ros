Tools_interface
====================================

| This package handles Niryo's tools.

Tools interface node
--------------------------
The ROS Node is made to:
 - Set and control tools.
 - Publish tool connection state.

Parameters - tools interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``check_tool_connection_frequency``
      -  | Publishes and controls rate for tools connection state.
         | Default: '2.0'
   *  -  ``id_list``
      -  | List of tools id
         | Default: '[11,12,13,14,31]'
   *  -  ``motor_type_list``
      -  | List of motor tools type 
         | Default: '["xl320","xl320","xl320","xl320","xl320"]'

Published topics - tools interface
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

Services - tools interface
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
      -  :ref:`tools_interface/PingDxlTool<source/stack_hardware/tools_interface:PingDxlTool (Service)>`
      -  Scans and sets for a tool plugged
   *  -  ``niryo_robot/tools/open_gripper``
      -  :ref:`tools_interface/OpenGripper<source/stack_hardware/tools_interface:OpenGripper (Service)>`
      -  Opens a gripper tool
   *  -  ``niryo_robot/tools/close_gripper``
      -  :ref:`tools_interface/CloseGripper<source/stack_hardware/tools_interface:CloseGripper (Service)>`
      -  Closes a gripper tool
   *  -  ``niryo_robot/tools/pull_air_vacuum_pump``
      -  :ref:`tools_interface/PullAirVacuumPump<source/stack_hardware/tools_interface:PullAirVacuumPump (Service)>`
      -  Pulls vacuum pump tool
   *  -  ``niryo_robot/tools/push_air_vacuum_pump``
      -  :ref:`tools_interface/PushAirVacuumPump<source/stack_hardware/tools_interface:PushAirVacuumPump (Service)>`
      -  Pushes vacuum pump tool
   *  -  ``niryo_robot/tools/reboot``
      -  :std_srvs:`Trigger`
      -  Reboots the motor of the equipped tool

Dependencies - tools interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :msgs_index:`std_srvs`
- :doc:`dynamixel_driver`

Services & messages files - tools interface
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
