Niryo Robot Commander Package
========================================

| This package is the one dealing with all commander related stuff.
| It is composed of only one node, which is running separately the arm commander and the tool commander


Commander Node
--------------------------
The ROS Node is made to interact with :
 - The Arm through MoveIt!
 - The tolls through the tool controller

All commands are firstly received on the actionlib server which
 * Handles concurrent requests
 * Checks if the command can't be processed due to other factors (ex: learning mode)
 * Validates parameters
 * Calls required controllers and returns appropriate status and message

The namespace used is : |namespace_emphasize|

Parameters - Commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Commander's Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``reference_frame``
      -  | Reference frame used by MoveIt! for moveP.
         | Default : 'world'
   *  -  ``move_group_commander_name``
      -  Name of the group that MoveIt is controlling. By default : "arm"
   *  -  ``jog_timer_rate_sec``
      -  Publish rate for jog controller
   *  -  ``simu_gripper``
      -  If you are using the simulated gripper and want to control the gripper


Action Server - Commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Commander Package Action Servers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``robot_action``
      -  :ref:`RobotMove<RobotMove (Action)>`
      -  Command the arm and tools through an action server

Services - Commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Commander Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``is_active``
      -  :ref:`GetBool`
      -  Indicate whereas a command is actually running or not
   *  -  ``stop_command``
      -  :ref:`Trigger`
      -  Stop the actual command
   *  -  ``set_max_velocity_scaling_factor``
      -  :ref:`SetInt`
      -  Set a percentage of maximum speed
   *  -  ``/niryo_robot/kinematics/forward``
      -  :ref:`GetFK<GetFK (Service)>`
      -  Compute a Forward Kinematic
   *  -  ``/niryo_robot/kinematics/inverse``
      -  :ref:`GetIK<GetIK (Service)>`
      -  Compute a Inverse Kinematic

Messages - Commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Commander Package Messages
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  :ref:`ArmMoveCommand<ArmMoveCommand (Message)>`
      -  Message to command the arm
   *  -  :ref:`RobotCommand<RobotCommand (Message)>`
      -  Message to command the robot (arm + tool)
   *  -  :ref:`ShiftPose<ShiftPose (Message)>`
      -  Message for shifting pose
   *  -  :ref:`PausePlanExecution<PausePlanExecution (Message)>`
      -  Pause movement execution

All these services are available as soon as the node is started

Dependencies - Commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`actionlib`
- :msgs_index:`actionlib_msgs`
- :msgs_index:`control_msgs`
- :msgs_index:`geometry_msgs`
- `MoveIt! <https://moveit.ros.org/>`_
- :msgs_index:`moveit_msgs`
- :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`
- :ref:`niryo_robot_tools <Niryo Robot Tools Package>`
- `python-numpy <https://numpy.org/>`_
- :wiki_ros:`ros_controllers`
- :wiki_ros:`rosbridge_server`
- :msgs_index:`sensor_msgs`
- :msgs_index:`std_msgs`
- :wiki_ros:`tf2_web_republisher`
- :msgs_index:`trajectory_msgs`


Action, Services & Messages files - Commander
------------------------------------------------------

RobotMove (Action)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/action/RobotMove.action
   :language: rostype

GetFK (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/srv/GetFK.srv
   :language: rostype

GetIK (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/srv/GetIK.srv
   :language: rostype

JogShift (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/srv/JogShift.srv
   :language: rostype

ArmMoveCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/msg/ArmMoveCommand.msg
   :language: rostype


PausePlanExecution (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/msg/PausePlanExecution.msg
   :language: rostype


RobotCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/msg/RobotCommand.msg
   :language: rostype


ShiftPose (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_commander/msg/ShiftPose.msg
   :language: rostype



.. |namespace| replace:: /niryo_robot_commander/
.. |namespace_emphasize| replace:: ``/niryo_robot_commander/``
.. |package_path| replace:: ../../../niryo_robot_commander
