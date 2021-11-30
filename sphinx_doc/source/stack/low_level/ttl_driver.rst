TTL Driver
===================================

| This package handles communication on TTL bus via dynamixel sdk.
| It provides an interface to :wiki_ros:`dynamixel_sdk`.

TTL Driver Node
--------------------------
The ROS Node is made to:
 - Send commands to any hardware on TTL bus
 - Receive data from the TTL bus

Parameters
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
         | Default: '100.0'
   *  -  ``ttl_hardware_write_frequency``
      -  | Write frequency.
         | Default: '50.0'
   *  -  ``ttl_hardware_read_data_frequency``
      -  | Read frequency.
         | Default: '15.0'
   *  -  ``ttl_hardware_read_end_effector_frequency``
      -  | Read end effector status frequency.
         | Default: '10.0'   
   *  -  ``ttl_hardware_read_status_frequency``
      -  | Read hardware status frequency.
         | Default: '0.5'

Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`dynamixel_sdk` 
- :doc:`../high_level/niryo_robot_msgs`

Services, Topics and Messages
-------------------------------------------------

.. list-table:: TTL Driver Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``niryo_robot/ttl_driver/set_dxl_leds``
      -  :ref:`source/high_level/niryo_robot_msgs:SetInt`
      -  Control dynamixels LED
   *  -  ``niryo_robot/ttl_driver/send_custom_value``
      -  :ref:`ttl_driver/SendCustomValue<source/stack_hardware/ttl_driver:SendCustomValue (Service)>`
      -  Send a custom command on the TTL bus
   *  -  ``niryo_robot/ttl_driver/read_custom_value``
      -  :ref:`ttl_driver/ReadCustomValue<source/stack_hardware/ttl_driver:ReadCustomValue (Service)>`
      -  Reads a custom register on the TTL bus
   *  -  ``niryo_robot/ttl_driver/write_pid_value``
      -  :ref:`ttl_driver/WritePIDValue<source/stack_hardware/ttl_driver:WritePIDValue (Service)>`
      -  Writes PID values to a dynamixel motor on the TTL Bus
   *  -  ``niryo_robot/ttl_driver/read_pid_value``
      -  :ref:`ttl_driver/ReadPIDValue<source/stack_hardware/ttl_driver:ReadPIDValue (Service)>`
      -  Reads PID values from a dynamixel motor on the TTL Bus
   *  -  ``niryo_robot/ttl_driver/write_velocity_profile``
      -  :ref:`ttl_driver/WriteVelocityProfile<source/stack_hardware/ttl_driver:WriteVelocityProfile (Service)>`
      -  Writes velocity and acceleration profiles to a stepper motor on the TTL Bus
   *  -  ``niryo_robot/ttl_driver/read_velocity_profile``
      -  :ref:`ttl_driver/ReadVelocityProfile<source/stack_hardware/ttl_driver:ReadVelocityProfile (Service)>`
      -  Reads velocity and acceleration profiles from a stepper motor on the TTL Bus

Services files
--------------------------------------------------

SendCustomValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/SendCustomValue.srv
   :language: rostype

ReadCustomValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadCustomValue.srv
   :language: rostype

WritePIDValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/WritePIDValue.srv
   :language: rostype

ReadPIDValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadPIDValue.srv
   :language: rostype

WriteVelocityProfile (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/WriteVelocityProfile.srv
   :language: rostype

ReadVelocityProfile (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/ttl_driver/srv/ReadVelocityProfile.srv
   :language: rostype


Messages files
--------------------------------------------------

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
