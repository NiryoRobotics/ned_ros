Niryo robot fake interface package
===================================

| This package provides fakes hardware interface when the robot is used in simulation.

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
      -  :ref:`SetInt`
      -  Start motors calibration - value can be 1 for auto calibration, 2 for manual
   *  -  ``/niryo_robot/joints_interface/request_new_calibration``
      -  :ref:`Trigger`
      -  Unset motors calibration
   *  -  ``niryo_robot/learning_mode/activate``
      -  :ref:`Trigger`
      -  Either activate or deactivate learning mode
   *  -  ``niryo_robot/tools/ping_and_set_dxl_tool``
      -  :ref:`tools_interface/PingDxlTool<PingDxlTool (Service)>`
      -  Scan and set for a tool plugged
   *  -  ``niryo_robot/tools/open_gripper``
      -  :ref:`tools_interface/OpenGripper<OpenGripper (Service)>`
      -  Open a Gripper tool
   *  -  ``niryo_robot/tools/close_gripper``
      -  :ref:`tools_interface/CloseGripper<CloseGripper (Service)>`
      -  Close a Gripper tool
   *  -  ``niryo_robot/tools/pull_air_vacuum_pump``
      -  :ref:`tools_interface/PullAirVacuumPump<PullAirVacuumPump (Service)>`
      -  Pull Vacuum Pump tool
   *  -  ``niryo_robot/tools/push_air_vacuum_pump``
      -  :ref:`tools_interface/PushAirVacuumPump<PushAirVacuumPump (Service)>`
      -  Push Vacuum Pump tool
   *  -  ``/niryo_robot/conveyor/control_conveyor``
      -  :ref:`ControlConveyor<ControlConveyor (Service)>`
      -  Send a command to the desired Conveyor Belt
   *  -  ``/niryo_robot/conveyor/ping_and_set_conveyor``
      -  :ref:`SetConveyor<SetConveyor (Service)>`
      -  Scan and set a new Conveyor Belt

Dependencies - fake interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- :msgs_index:`std_msgs`
- :wiki_ros:`hardware_interface <hardware_interface>`
- :wiki_ros:`controller_manager <controller_manager>`
- :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`
- :ref:`tools_interface <Niryo Robot Tools Interface Package>`
- :ref:`joints_interface <Niryo Robot Joints Interface Package>`
- :ref:`conveyor_interface <Niryo Robot Conveyor Interface Package>` 
