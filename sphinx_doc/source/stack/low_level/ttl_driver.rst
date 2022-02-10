TTL Driver
=====================================

| This package handles motors which communicate via the protocol TTL.

| This package is based on the DXL SDK. It provides an interface to :wiki_ros:`dynamixel_sdk`.

TTL Driver Node (For only the development and debugging propose)
-------------------------------------------------------------------
The ROS Node is made to:
 - Initialize TTL Interface.
 - Get configuration of motors and add to TTL Interface.

TTL Interface Core
--------------------------

It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been conceived to:
 - Initialize the TTL Interface (Interface used by other packages) and physical bus with the configurations.
 - Add, remove and monitor devices.
 - Start getting data and sending data on the physical bus.
 - Start ROS stuffs like services, topics.

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters - TTL Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. note::
   These configuration parameters are chosen and tested many times to work correctly. 
   Please make sure that you understand what you do before editing these files.

.. list-table:: TTL Driver's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``ttl_hardware_control_loop_frequency``
      -  | Frequency of the bus control loop.
         | Default: '240.0'
   *  -  ``ttl_hardware_write_frequency``
      -  | Writes frequency on the bus.
         | Default: '120.0'
   *  -  ``ttl_hardware_read_data_frequency``
      -  | Reads frequency on the bus.
         | Default: '120.0'
   *  -  ``ttl_hardware_read_status_frequency``
      -  | Reads frequency for device status on the bus.
         | Default: '0.7'
   *  -  ``ttl_hardware_read_end_effector_frequency``
      -  | Read frequency for End Effector's status.
         | Default: '13.0'
   *  -  ``bus_params/Baudrate``
      -  | Baudrates of TTL bus
         | Default: '1000000'
   *  -  ``bus_params/uart_device_name``
      -  | Name of UART port using
         | Default: '/dev/ttyAMA0'

Dependencies - TTL Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`dynamixel_sdk` 
- :doc:`../high_level/niryo_robot_msgs`
- :doc:`common`
- :msgs_index:`std_msgs`

Services - TTL Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: TTL Driver Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``niryo_robot/ttl_driver/set_dxl_leds``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:SetInt`
      -  Controls dynamixel LED
   *  -  ``niryo_robot/ttl_driver/send_custom_value``
      -  :ref:`SendCustomValue<source/stack/low_level/ttl_driver:SendCustomValue (Service)>`
      -  Writes data at a custom register address of a given TTL device
   *  -  ``niryo_robot/ttl_driver/read_custom_value``
      -  :ref:`ReadCustomValue<source/stack/low_level/ttl_driver:ReadCustomValue (Service)>`
      -  Reads data at a custom register address of a given TTL device
   *  -  ``niryo_robot/ttl_driver/read_pid_value``
      -  :ref:`ReadPIDValue<source/stack/low_level/ttl_driver:ReadPIDValue (Service)>`
      -  Reads the PID of dxl motors
   *  -  ``niryo_robot/ttl_driver/write_pid_value``
      -  :ref:`WritePIDValue<source/stack/low_level/ttl_driver:WritePIDValue (Service)>`
      -  Writes the PID for dxl motors
   *  -  ``niryo_robot/ttl_driver/read_velocity_profile``
      -  :ref:`ReadVelocityProfile<source/stack/low_level/ttl_driver:ReadVelocityProfile (Service)>`
      -  Reads velocity Profile for steppers
   *  -  ``niryo_robot/ttl_driver/write_velocity_profile``
      -  :ref:`WriteVelocityProfile<source/stack/low_level/ttl_driver:WriteVelocityProfile (Service)>`
      -  Writes velocity Profile for steppers


Services & Messages files - TTL Driver
--------------------------------------------------

SendCustomValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/WriteCustomValue.srv
   :language: rostype

ReadCustomValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadCustomValue.srv
   :language: rostype

ReadPIDValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadPIDValue.srv
   :language: rostype

WritePIDValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadPIDValue.srv
   :language: rostype

ReadVelocityProfile (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadVelocityProfile.srv
   :language: rostype

WriteVelocityProfile (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/WriteVelocityProfile.srv
   :language: rostype

MotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/msg/MotorHardwareStatus.msg
   :language: rostype

MotorCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/msg/MotorCommand.msg
   :language: rostype

ArrayMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/msg/ArrayMotorHardwareStatus.msg
   :language: rostype


.. |namespace_cpp| replace:: ttl_driver
.. |namespace| replace:: /niryo_robot/ttl_driver/
.. |namespace_emphasize| replace:: ``/niryo_robot/ttl_driver/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/ttl_driver
