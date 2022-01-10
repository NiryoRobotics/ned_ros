Niryo_robot_arm_commander
========================================

| This package is the one dealing with all commander related stuff.

| It is composed of only one node, which runs separately the arm commander and the tool commander.


Commander node
--------------------------
The ROS Node is made to interact with:
 - The arm through MoveIt!
 - The tools through the tool controller.

All commands are firstly received on the actionlib server which:
 * Handles concurrent requests.
 * Checks if the command can't be processed due to other factors (ex: learning mode).
 * Validates parameters.
 * Calls required controllers and returns appropriate status and message.

It belongs to the ROS namespace: |namespace_emphasize|.

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
      -  Name of the group that MoveIt is controlling. By default: "arm"
   *  -  ``jog_timer_rate_sec``
      -  Publish rate for jog controller
   *  -  ``simu_gripper``
      -  If you are using the simulated Gripper and want to control the Gripper


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
      -  :ref:`RobotMove<source/stack/high_level/niryo_robot_arm_commander:RobotMove>`
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
      -  :ref:`source/stack/high_level/niryo_robot_msgs:GetBool`
      -  Indicate whereas a command is actually running or not
   *  -  ``stop_command``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Stop the actual command
   *  -  ``set_max_velocity_scaling_factor``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:SetInt`
      -  Set a percentage of maximum speed
   *  -  ``/niryo_robot/kinematics/forward``
      -  :ref:`GetFK<source/stack/high_level/niryo_robot_arm_commander:GetFK>`
      -  Compute a Forward Kinematic
   *  -  ``/niryo_robot/kinematics/inverse``
      -  :ref:`GetIK<source/stack/high_level/niryo_robot_arm_commander:GetIK>`
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
   *  -  :ref:`ArmMoveCommand<source/stack/high_level/niryo_robot_arm_commander:ArmMoveCommand>`
      -  Message to command the arm
   *  -  :ref:`ShiftPose<source/stack/high_level/niryo_robot_arm_commander:ShiftPose>`
      -  Message for shifting pose
   *  -  :ref:`PausePlanExecution<source/stack/high_level/niryo_robot_arm_commander:PausePlanExecution>`
      -  Pause movement execution

All these services are available as soon as the node is started.

Dependencies - Commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`actionlib`
- :msgs_index:`actionlib_msgs`
- :msgs_index:`control_msgs`
- :msgs_index:`geometry_msgs`
- `MoveIt! <https://moveit.ros.org/>`_
- :msgs_index:`moveit_msgs`
- :doc:`niryo_robot_msgs`
- :doc:`niryo_robot_tools_commander`
- `python-numpy <https://numpy.org/>`_
- :wiki_ros:`ros_controllers`
- :wiki_ros:`rosbridge_server`
- :msgs_index:`sensor_msgs`
- :msgs_index:`std_msgs`
- :wiki_ros:`tf2_web_republisher`
- :msgs_index:`trajectory_msgs`


Action files - Commander
------------------------------------------------------

RobotMove
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/action/RobotMove.action
   :language: rostype


Services files - Commander
------------------------------------------------------

GetFK
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/srv/GetFK.srv
   :language: rostype

GetIK
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/srv/GetIK.srv
   :language: rostype

JogShift
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/srv/JogShift.srv
   :language: rostype


Messages files - Commander
------------------------------------------------------

ArmMoveCommand
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/msg/ArmMoveCommand.msg
   :language: rostype


PausePlanExecution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/msg/PausePlanExecution.msg
   :language: rostype


ShiftPose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_arm_commander/msg/ShiftPose.msg
   :language: rostype



.. |namespace| replace:: /niryo_robot_arm_commander/
.. |namespace_emphasize| replace:: ``/niryo_robot_arm_commander/``
.. |package_path| replace:: ../../../../niryo_robot_arm_commander
