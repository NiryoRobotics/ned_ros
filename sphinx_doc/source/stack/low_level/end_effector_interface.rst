End Effector Interface
=====================================

| This package handles Niryo’s End Effector, it is supported from Ned 2.
| The End Effector can be used to do many things like activating Free Motion, Calibration and much more.

End Effector Interface node (For development and debugging)
------------------------------------------------------------
The ROS Node in End Effector Interface Package is used to:
 - Create :doc:ttl_driver to communicate with hardware.
 - Initialize End Effector Interface.

End Effector Interface Core
-----------------------------
This interface is integrated in to :doc:niryo_robot_hardware_interface package.

The missions of End Effector Interfaces:
 - Get TTL Driver.
 - Initialize End Effector parameters.
 - Get information about End Effector sent from TTL driver.
 - Begin ROS stuffs like publishing the status of buttons, starting service on IO State.

The namespace used is: ``/niryo_robot_hardware_interface/end_effector_interface/``

Parameters - End Effector Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: end_effector_interface's Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center
    
   *  -  Name
      -  Description
   *  -  ``end_effector_id``
      -  | Id of End Effector in TTL bus
         | Default: 0
   *  -  ``check_end_effector_status_frequency``
      -  | Frequency to get End Effector from driver
         | Default: 40.0
   *  -  ``button_2__type``
      -  | Button used to activate FreeMotion
         | Default: free_drive
   *  -  ``button_1__type``
      -  | Button used to save the actual position of robot
         | Default: save_position
   *  -  ``button_0__type``
      -  | Custom Button used by Users to do something
         | Default: custom
   *  -  ``hardware_type``
      -  | Type of End Effector. It can be end_effector or fake_end_effector
         | Default: end_effector


Published topics - End Effector Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: end_effector_interface Package Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center
   
   *  -  Name
      -  Message Type
      -  Description
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_free_drive_button_state_publisher
      -  :ref:`EEButtonStatus<source/stack/low_level/end_effector_interface:EEButtonStatus (Message)>`
      -  Publish state of Free Driver Button
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_save_button_state_publisher
      -  :ref:`EEButtonStatus<source/stack/low_level/end_effector_interface:EEButtonStatus (Message)>`
      -  Publish state of Save Position Button
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_custom_button_state_publisher
      -  :ref:`EEButtonStatus<source/stack/low_level/end_effector_interface:EEButtonStatus (Message)>`
      -  Publish state of Custom Button
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_digital_out_publisher
      -  :ref:`EEIOState<source/stack/low_level/end_effector_interface:EEIOState (Message)>`
      -  Publish state of IO Digital

Services - End Effector Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: end_effector_interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center
   
   *  -  Name
      -  Service Type
      -  Description
   *  -  set_ee_io_state
      -  :ref:`SetEEDigitalOut<source/stack/low_level/end_effector_interface:SetEEDigitalOut (Service)>`
      -  Set up digital output on End Effector

Dependencies - End Effector Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- :msgs_index:`std_msgs`
- :doc:`ttl_driver`

Add dependencies of package

Services & Messages files - End Effector Interface
--------------------------------------------------

SetEEDigitalOut (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/end_effector_interface/srv/SetEEDigitalOut.srv
   :language: rostype

EEButtonStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/end_effector_interface/msg/EEButtonStatus.msg
   :language: rostype

EEIOState (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/end_effector_interface/msg/EEIOState.msg
   :language: rostype

