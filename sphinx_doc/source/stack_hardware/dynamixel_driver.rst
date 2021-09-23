Dynamixel_driver
=====================================

| This package handles dynamixel motors communication through dynamixel sdk.
| It provides an interface to :wiki_ros:`dynamixel_sdk`.

Dynamixel Driver Node
--------------------------
The ROS Node is made to:
 - Send commands to dynamixel motors
 - Receive dynamixel motors data

Parameters - Dynamixel Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Dynamixel Driver's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``dxl_hardware_control_loop_frequency``
      -  | Controls loop frequency.
         | Default: '100.0'
   *  -  ``dxl_hardware_write_frequency``
      -  | Writes frequency.
         | Default: '50.0'
   *  -  ``dxl_hardware_read_data_frequency``
      -  | Reads frequency.
         | Default: '15.0'
   *  -  ``dxl_hardware_read_status_frequency``
      -  | Reads dynamixels status frequency.
         | Default: '0.5'
   *  -  ``dxl_hardware_check_connection_frequency``
      -  | Checks dynamixels connection frequency.
         | Default: '2.0'
   *  -  ``dxl_motor_id_list``
      -  | List of dynamixels ID
         | Default: '[2,3,6]'
   *  -  ``dxl_motor_type_list``
      -  | List of dynamixels type
         | Default: '["xl430","xl430","xl320"]'

Services - Dynamixel Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Dynamixel Driver Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``niryo_robot/dynamixel_driver/set_dxl_leds``
      -  :ref:`source/ros/niryo_robot_msgs:SetInt`
      -  Controls dynamixel LED
   *  -  ``niryo_robot/dynamixel_driver/send_custom_dxl_value``
      -  :ref:`dynamixel_driver/SendCustomDxlValue<source/stack_hardware/dynamixel_driver:SendCustomDxlValue (Service)>`
      -  Sends a custom dynamixel command

Dependencies - Dynamixel Driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`dynamixel_sdk` 
- :doc:`../ros/niryo_robot_msgs`

Services & Messages files - Dynamixel Driver
--------------------------------------------------

SendCustomDxlValue (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/dynamixel_driver/srv/SendCustomDxlValue.srv
   :language: rostype

DxlMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/dynamixel_driver/msg/DxlMotorHardwareStatus.msg
   :language: rostype

DxlMotorCommand (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/dynamixel_driver/msg/DxlMotorCommand.msg
   :language: rostype

DxlArrayMotorHardwareStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/dynamixel_driver/msg/DxlArrayMotorHardwareStatus.msg
   :language: rostype
