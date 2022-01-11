CAN Driver
===================================

| This package provides an interface between high level ROS packages and handler of CAN Bus. It uses the mcp_can_rpi for CAN bus communication.

| It is used by only Ned and the Niryo One.

CAN Driver Node (For only the development and debugging propose)
-------------------------------------------------------------------
The ROS Node is made to:
 - Initialize CAN Interface.

CAN Interface Core
--------------------------
It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been conceived to:
 - Initialize the CAN Interface and physical bus with the configurations.
 - Add, remove and monitor devices on bus.
 - Start control loop to get and send data from/to motors.
 - Start ROS stuffs like services, topics if they exist.

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. note::
   These configuration parameters are set to work with Niryo's robot. Do not edit them.

.. list-table:: CAN Driver's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``can_hardware_control_loop_frequency``
      -  | Control loop frequency.
         | Default: '1500.0'
   *  -  ``can_hw_write_frequency``
      -  | Write frequency.
         | Default: '200.0'
   *  -  ``can_hw_read_frequency``
      -  | Read frequency.
         | Default: '50.0'
   *  -  ``bus_params/spi_channel``
      -  | spi channel.
         | Default: '0'
   *  -  ``bus_params/spi_baudrate``
      -  | Baudrate.
         | Default: '1000000'
   *  -  ``bus_params/gpio_can_interrupt``
      -  | GPIO Interrupt.
         | Default: '25'

Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :doc:`../third_parties/mcp_can_rpi`
- :doc:`../high_level/niryo_robot_msgs`
- :doc:`common`
- :msgs_index:`std_msgs`

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

.. |namespace_cpp| replace:: can_driver
.. |namespace| replace:: /can_driver/
.. |namespace_emphasize| replace:: ``/can_driver/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/can_driver
