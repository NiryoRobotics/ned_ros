Tools Interface
====================================

| This package handles Niryo's tools.

Tools interface node
--------------------------
The ROS Node is made to:
 - Set and control tools for better usage.
 - Publish tool connection state.

Parameters
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

Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_srvs`
- :msgs_index:`std_msgs`
- :msgs_index:`std_srvs`
- :doc:`ttl_driver`

Services, Topics and Messages
-------------------------------------------------

Published topics
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

Services
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
      -  :ref:`tools_interface/PingDxlTool<source/stack/low_level/tools_interface:PingDxlTool (Service)>`
      -  Scans and sets for a tool plugged
   *  -  ``niryo_robot/tools/open_gripper``
      -  :ref:`tools_interface/OpenGripper<source/stack/low_level/tools_interface:ToolCommand (Service)>`
      -  Opens the gripper
   *  -  ``niryo_robot/tools/close_gripper``
      -  :ref:`tools_interface/OpenGripper<source/stack/low_level/tools_interface:ToolCommand (Service)>`
      -  Closes the gripper
   *  -  ``niryo_robot/tools/pull_air_vacuum_pump``
      -  :ref:`tools_interface/OpenGripper<source/stack/low_level/tools_interface:ToolCommand (Service)>`
      -  Pulls air with the vacuum pump
   *  -  ``niryo_robot/tools/push_air_vacuum_pump``
      -  :ref:`tools_interface/OpenGripper<source/stack/low_level/tools_interface:ToolCommand (Service)>`
      -  Pushes air with the vacuum pump
   *  -  ``niryo_robot/tools/reboot``
      -  :std_srvs:`Trigger`
      -  Reboots the motor of the equipped tool


PingDxlTool (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/tools_interface/srv/PingDxlTool.srv
   :language: rostype

ToolCommand (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/tools_interface/srv/ToolCommand.srv
   :language: rostype

