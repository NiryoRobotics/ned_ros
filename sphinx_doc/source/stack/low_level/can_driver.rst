CAN Driver
===================================

| This package provides an interface between high level ROS packages and motors on CAN bus;

CAN Driver Node
--------------------------
The ROS Node is made to:
 - Send commands to stepper motors.
 - Receive stepper motors data.

Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Stepper Driver's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``can_hardware_control_loop_frequency``
      -  | Controls loop frequency.
         | Default: '100.0'
   *  -  ``can_hw_write_frequency``
      -  | Write frequency.
         | Default: '50.0'
   *  -  ``can_hw_read_frequency``
      -  | Read frequency.
         | Default: '50.0'


Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :doc:`../third_parties/mcp_can_rpi`
- :doc:`../../stack/high_level/niryo_robot_msgs`
- :doc:`common`


Services, Topics and Messages
-------------------------------------------------

StepperCmd (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/can_driver/srv/StepperCmd.srv
   :language: rostype

StepperMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/can_driver/msg/StepperMotorHardwareStatus.msg
   :language: rostype

StepperMotorCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/can_driver/msg/StepperMotorCommand.msg
   :language: rostype

StepperArrayMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/can_driver/msg/StepperArrayMotorHardwareStatus.msg
   :language: rostype
