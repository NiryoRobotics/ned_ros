Fake_interface
===================================

| This package provides fakes hardware interfaces when the robot is used in simulation.

Fake interface node
--------------------------
The ROS Node is made to simulate:
 - tools interface
 - Conveyor Belt interface
 - joints interface

Published topics - fake interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Fake Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/learning_mode/state``
      -  :std_msgs:`Bool`
      -  Learning mode state
   *  -  ``/niryo_robot_hardware/tools/current_id``
      -  :std_msgs:`Int32`
      -  Current tool ID

Services - fake interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Fake Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/joints_interface/calibrate_motors``
      -  :ref:`source/ros/niryo_robot_msgs:SetInt`
      -  Starts motors calibration - value can be 1 for auto calibration, 2 for manual
   *  -  ``/niryo_robot/joints_interface/request_new_calibration``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Unsets motors calibration
   *  -  ``niryo_robot/learning_mode/activate``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Either activates or deactivates learning mode
   *  -  ``niryo_robot/tools/ping_and_set_dxl_tool``
      -  :ref:`tools_interface/PingDxlTool<source/stack_hardware/tools_interface:PingDxlTool (Service)>`
      -  Scans and sets for a tool plugged
   *  -  ``niryo_robot/tools/open_gripper``
      -  :ref:`tools_interface/OpenGripper<source/stack_hardware/tools_interface:OpenGripper (Service)>`
      -  Opens a Gripper tool
   *  -  ``niryo_robot/tools/close_gripper``
      -  :ref:`tools_interface/CloseGripper<source/stack_hardware/tools_interface:CloseGripper (Service)>`
      -  Closes a Gripper tool
   *  -  ``niryo_robot/tools/pull_air_vacuum_pump``
      -  :ref:`tools_interface/PullAirVacuumPump<source/stack_hardware/tools_interface:PullAirVacuumPump (Service)>`
      -  Pulls Vacuum Pump tool
   *  -  ``niryo_robot/tools/push_air_vacuum_pump``
      -  :ref:`tools_interface/PushAirVacuumPump<source/stack_hardware/tools_interface:PushAirVacuumPump (Service)>`
      -  Pushes Vacuum Pump tool
   *  -  ``/niryo_robot/conveyor/control_conveyor``
      -  :ref:`ControlConveyor<source/stack_hardware/conveyor_interface:ControlConveyor (Service)>`
      -  Sends a command to the desired Conveyor Belt
   *  -  ``/niryo_robot/conveyor/ping_and_set_conveyor``
      -  :ref:`SetConveyor<source/stack_hardware/conveyor_interface:SetConveyor (Service)>`
      -  Scans and sets a new Conveyor Belt

Dependencies - fake interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- :msgs_index:`std_msgs`
- :wiki_ros:`hardware_interface <hardware_interface>`
- :wiki_ros:`controller_manager <controller_manager>`
- :doc:`../ros/niryo_robot_msgs`
- :doc:`tools_interface`
- :doc:`joints_interface`
- :doc:`conveyor_interface` 
