Niryo Robot Stepper Driver Package
===================================

| This package handles stepper motors communication 

Stepper Driver Node
--------------------------
The ROS Node is made to :
 - Send commands to stepper motor
 - Receive stepper motors data

Parameters - Stepper Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Stepper Driver's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``can_hardware_control_loop_frequency``
      -  | control loop frequency.
         | Default : '1500.0'
   *  -  ``can_hw_write_frequency``
      -  | Write frequency.
         | Default : '50.0'
   *  -  ``can_hw_check_connection_frequency``
      -  | Check can connection frequency.
         | Default : '2.0'
   *  -  ``stepper_motor_id_list``
      -  | list of steppers ID
         | Default : '[1,2,3]'


Dependencies - Stepper Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- `mcp_can_rpi <https://github.com/coryjfowler/MCP_CAN_lib>`_
- :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`

Services & Messages files - Stepper Driver
-------------------------------------------------

StepperMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/stepper_driver/msg/StepperMotorHardwareStatus.msg
   :language: rostype

StepperMotorCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/stepper_driver/msg/StepperMotorCommand.msg
   :language: rostype

StepperArrayMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/stepper_driver/msg/StepperArrayMotorHardwareStatus.msg
   :language: rostype
