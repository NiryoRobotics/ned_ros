TTL Driver
=====================================

| This package handles motors which communicate via the protocol TTL.
| This package is based on the DXL SDK.
| It provides an interface to :wiki_ros:`dynamixel_sdk`.

TTL Driver Node (For Development and Debugging)
--------------------------------------------------
The ROS Node is made to:
 - Initialize TTL Interface.
 - Get configuration of motors and add to TTL Interface

TTL Interface Core
--------------------------

It is integrated in :doc:`niryo_robot_hardware_interface` package
 - Initialize the TTL Interface and TTL bus with the configuration.
 - Add or remove devices using protocol TTL to the Interface from other packages.
 - Start getting data and sending data on the physical bus.
 - Start ROS stuffs like services, topic.

Parameters - TTL Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: TTL Driver's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``ttl_hardware_control_loop_frequency``
      -  | Controls loop frequency.
         | Default: '600.0'
   *  -  ``ttl_hardware_write_frequency``
      -  | Writing frequency.
         | Default: '200.0'
   *  -  ``ttl_hardware_read_data_frequency``
      -  | Reading frequency.
         | Default: '300.0'
   *  -  ``ttl_hardware_read_status_frequency``
      -  | Reads Ttl's device status frequency.
         | Default: '0.7'
   *  -  ``ttl_hardware_read_end_effector_frequency``
      -  | Reads End Effector's status frequency.
         | Default: '13.0'
   *  -  ``bus_params/Baudrate``
      -  | Baudrate of TTL bus
         | Default: '1000000'
   *  -  ``bus_params/uart_device_name``
      -  | name of uart port using
         | Default: '/dev/ttyAMA0'

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
      -  Control dynmixel LED
   *  -  ``niryo_robot/ttl_driver/send_custom_value``
      -  :ref:`SendCustomValue<source/stack/low_level/ttl_driver:SendCustomValue (Service)>`
      -  Send a custom command to Ttl device
   *  -  ``niryo_robot/ttl_driver/read_custom_value``
      -  :ref:`ReadCustomValue<source/stack/low_level/ttl_driver:ReadCustomValue (Service)>`
      -  Read a custom command to Ttl device
   *  -  ``niryo_robot/ttl_driver/read_pid_value``
      -  :ref:`ReadPIDValue<source/stack/low_level/ttl_driver:ReadPIDValue (Service)>`
      -  Read the PID of dxl motors
   *  -  ``niryo_robot/ttl_driver/write_pid_value``
      -  :ref:`WritePIDValue<source/stack/low_level/ttl_driver:WritePIDValue (Service)>`
      -  Write the PID for dxl motors
   *  -  ``niryo_robot/ttl_driver/read_velocity_profile``
      -  :ref:`ReadVelocityProfile<source/stack/low_level/ttl_driver:ReadVelocityProfile (Service)>`
      -  Read velocity Profile for steppers
   *  -  ``niryo_robot/ttl_driver/write_velocity_profile``
      -  :ref:`WriteVelocityProfile<source/stack/low_level/ttl_driver:WriteVelocityProfile (Service)>`
      -  Write velocity Profile for steppers

Dependencies - TTL Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`dynamixel_sdk` 
- :doc:`../../stack/high_level/niryo_robot_msgs`
- :msgs_index:`std_msgs`

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

CollisionStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/msg/CollisionStatus.msg
   :language: rostype
