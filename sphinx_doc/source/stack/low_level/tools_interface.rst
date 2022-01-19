Tools Interface
====================================

| This package handles Niryo's tools.

Tools interface node (For Development and Debugging)
-----------------------------------------------------
The ROS Node is made to:
 - Initialize Tool Interface with configuration parameters.
 - Start ROS stuffs like services, topics.

Tools Interface Core
--------------------------

It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been conceived to:
 - Initialize the Tool Interface.
 - Provide services for setting and controlling tools.
 - Publish tool connection state.

It belongs to the ROS namespace: |namespace_emphasize|.

Tool Interface's default Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: default.yaml
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``check_tool_connection_frequency``
      -  | The frequency where tool interface check and publish the state of the tool connected,
         | or remove tool if it is disconnected.
         | Default: '2.0'

Tool Interface's hardware specific Parameters 
**************************************************

These parameters are specific to the hardware version (Ned, One or Ned2).
This file comes in a different version for each hardware version, located in a directory of the hardware version name.

.. list-table:: tools_params.yaml
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
      -  Supported Hardware versions
   *  -  ``id_list``
      -  | List of default IDs of each tool supported by Niryo
         | Default: '[11,12,13,30,31]'
      -  All Versions
   *  -  ``type_list``
      -  | List of motor tools type 
         | Default: 'xl320' for NED and ONE
         | Default: 'xl330' for NED2
         | Default: 'fakeDxl' for simulation
      -  All Versions
   *  -  ``name_list``
      -  | List of tools's name corresponds to ID list and type list above
         | Default: '["Standard Gripper", "Large Gripper", "Adaptive Gripper", "Vacuum Pump", "Electromagnet"]'
      -  All Versions
   

Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :msgs_index:`std_srvs`
- :doc:`ttl_driver`
- :doc:`common`

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


.. |namespace_cpp| replace:: tools_interface
.. |namespace| replace:: /tools_interface/
.. |namespace_emphasize| replace:: ``/tools_interface/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/tools_interface
