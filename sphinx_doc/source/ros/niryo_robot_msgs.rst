Niryo_robot_msgs
===========================================================

This package contains standard messages which can be used by all other packages.


Niryo messages
---------------------

.. list-table:: Ned Messages
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  :ref:`source/ros/niryo_robot_msgs:CommandStatus`
      -  Enum-wise message for status code
   *  -  :ref:`source/ros/niryo_robot_msgs:ObjectPose`
      -  x, y, z, roll, pitch, yaw
   *  -  :ref:`source/ros/niryo_robot_msgs:RobotState`
      -  position, rpy, quaternion
   *  -  :ref:`source/ros/niryo_robot_msgs:RPY`
      -  roll, pitch, yaw
   *  -  :ref:`source/ros/niryo_robot_msgs:HardwareStatus`
      -  several hardware informations 
   *  -  :ref:`source/ros/niryo_robot_msgs:SoftwareVersion`
      -  several software version 

Niryo services
----------------------

.. list-table:: Ned Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  :ref:`source/ros/niryo_robot_msgs:GetBool`
      -  Return a bool
   *  -  :ref:`source/ros/niryo_robot_msgs:GetInt`
      -  Return a integer
   *  -  :ref:`source/ros/niryo_robot_msgs:GetStringList`
      -  Return a list of string
   *  -  :ref:`source/ros/niryo_robot_msgs:SetBool`
      -  Set a bool and return status
   *  -  :ref:`source/ros/niryo_robot_msgs:SetInt`
      -  Set a integer and return status
   *  -  :ref:`source/ros/niryo_robot_msgs:SetString`
      -  Set a string and return status
   *  -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Trigger a task


Niryo message dependencies
-----------------------------------

- :msgs_index:`geometry_msgs`

Niryo message files
-----------------------------------

CommandStatus
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/CommandStatus.msg
   :language: rostype

ObjectPose
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/ObjectPose.msg
   :language: rostype

RobotState
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/RobotState.msg
   :language: rostype

RPY
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/RPY.msg
   :language: rostype

HardwareStatus
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/HardwareStatus.msg
   :language: rostype

SoftwareVersion
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/SoftwareVersion.msg
   :language: rostype

Niryo Service files
---------------------------

GetBool
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/GetBool.srv
   :language: rostype

GetInt
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/GetInt.srv
   :language: rostype

GetNameDescriptionList
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/GetNameDescriptionList.srv
   :language: rostype

GetStringList
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/GetStringList.srv
   :language: rostype

SetBool
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/SetBool.srv
   :language: rostype

SetInt
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/SetInt.srv
   :language: rostype

SetString
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/SetString.srv
   :language: rostype

Trigger
^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/Trigger.srv
   :language: rostype
