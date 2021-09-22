Niryo robot tools commander package
========================================

Provides functionalities to control end-effectors and accessories for Ned.

This package allows to manage the TCP (Tool Center Point) of the robot.
If the functionality is activated, all the movements (in Cartesian coordinates [x, y, z, roll, pitch, yaw]) of the robot will be performed according to this TCP.
The same program can then work with several tools by adapting the TCP transformation to them.
By default this feature is disabled, but can be enabled through the robot services.

Tools Commander node
--------------------------
The ROS Node is made of services to equip tool, an action server for tool command and topics for the current tool or the tool state.

The namespace used is: |namespace_emphasize|

Action server - tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Package Action Server
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``action_server``
      -  :ref:`ToolAction<source/ros/niryo_robot_tools_commander:ToolAction (Action)>`
      -  Command the tool through an action server

Publisher - tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Package Publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``current_id``
      -  :std_msgs:`Int32`
      -  Publish the current tool ID
   *  -  ``tcp``
      -  :ref:`TCP<source/ros/niryo_robot_tools_commander:TCP (Message)>`
      -  Publish if the TCP (Tool Center Point) is enabled and transformation between the tool_link and the TCP

Services - tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``update_tool``
      -  :std_srvs:`Trigger`
      -  Ping/scan for a dxl motor flashed with an ID corresponding to a tool and equip it (if found)
   *  -  ``equip_electromagnet``
      -  :ref:`source/ros/niryo_robot_msgs:SetInt`
      -  Equip the electromagnet with the motor ID given as parameter
   *  -  ``enable_tcp``
      -  :ref:`source/ros/niryo_robot_msgs:SetBool`
      -  | Enable or disable the TCP (Tool Center Point) functionality.
         | When we activate it, the transformation will be the last one saved since the robot started.
         | By default it will be the one of the equipped tool.
   *  -  ``set_tcp``
      -  :ref:`SetTCP<source/ros/niryo_robot_tools_commander:SetTCP (Service)>`
      -  Activate the TCP (Tool Center Point) functionality and defines a new TCP transformation.
   *  -  ``reset_tcp``
      -  :std_srvs:`Trigger`
      -  Reset the TCP transformation. By default it will be the one of the equipped tool.

Dependencies - tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^
* :doc:`niryo_robot_msgs`
* :msgs_index:`std_msgs`
* :msgs_index:`geometry_msgs`

Action files - tools
------------------------------------------------------

ToolAction (Action)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_tools_commander/action/Tool.action
   :language: rostype

Messages files - tools
------------------------------------------------------

ToolCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_tools_commander/msg/ToolCommand.msg
   :language: rostype

TCP (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_tools_commander/msg/TCP.msg
   :language: rostype

Services files - tools
------------------------------------------------------

SetTCP (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_tools_commander/srv/SetTCP.srv
   :language: rostype

.. |namespace| replace:: /niryo_robot_tools_commander/
.. |namespace_emphasize| replace:: ``/niryo_robot_tools_commander/``
.. |package_path| replace:: ../../../niryo_robot_tools_commander

