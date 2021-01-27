Niryo Robot Tools Package
========================================

Provides functionalities to control end-effectors and accessories for Ned.

Tools Node
--------------------------
The ROS Node is made of services to equip tool, an action server for tool command and topics for the current tool or the tool state.

The namespace used is : |namespace_emphasize|

Action Server - Tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Tools Package Action Server
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``robot_action``
      -  :ref:`ToolAction<ToolAction (Action)>`
      -  Command the tool through an action server

Publisher - Tools
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
      -  :std_msgs:`std_msgs/Int32<Int32>`
      -  Publish the current tool ID

Services - Tools
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
      -  :ref:`Trigger`
      -  Ping/scan for a dxl motor flashed with an ID corresponding to a tool and equip it (if found)
   *  -  ``equip_electromagnet``
      -  :ref:`SetInt`
      -  Equip the electromagnet with the motor ID given as parameter

Dependencies - Tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^
* :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`
* :msgs_index:`std_msgs`

Action, Services & Messages files - Tools
------------------------------------------------------

ToolAction (Action)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_tools/action/Tool.action
   :language: rostype

ToolCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_tools/msg/ToolCommand.msg
   :language: rostype

.. |namespace| replace:: /niryo_robot_tools/
.. |namespace_emphasize| replace:: ``/niryo_robot_tools/``
.. |package_path| replace:: ../../../niryo_robot_tools
