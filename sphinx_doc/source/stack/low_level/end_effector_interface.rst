End Effector Interface
=====================================

| This package handles the End Effector Panel of a robot, it is supported from Ned 2.
| It provides services and topics specific to the End Effector Panel in order to be used by a final user.

| However, it does not deal with the low level bus communication with the components: this is done in the :doc:`ttl_driver` package.

End Effector Interface node (For development and debug)
------------------------------------------------------------
The ROS Node in End Effector Interface Package is used to:
 - Instantiate a :doc:`ttl_driver` manager to communicate with hardware.
 - Initialize End Effector Interface.

End Effector Interface Core
-----------------------------
It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been conceived to:
 - Interface with TTL Driver.
 - Initialize End Effector parameters.
 - Retrieve End Effector data from TTL driver.
 - Publish the status of buttons.
 - Publish the collision detection status.
 - Start service on IO State.

It belongs to the ROS namespace: |namespace_emphasize|.

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
      -  | Id of the End Effector in TTL bus
         | Default: 0
   *  -  ``check_end_effector_status_frequency``
      -  | Frequency to get the End Effector from driver
         | Default: 40.0
   *  -  ``button_2__type``
      -  | Button used to activate the FreeMotion mode
         | Default: free_drive
   *  -  ``button_1__type``
      -  | Button used to save the actual position of the robot
         | Default: save_position
   *  -  ``button_0__type``
      -  | Custom Button used by users to do something
         | Default: custom
   *  -  ``hardware_type``
      -  | Type of the End Effector. It can be end_effector or fake_end_effector
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
      -  Publishes state of Free Motion Button
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_save_button_state_publisher
      -  :ref:`EEButtonStatus<source/stack/low_level/end_effector_interface:EEButtonStatus (Message)>`
      -  Publishes state of Save Position Button
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_custom_button_state_publisher
      -  :ref:`EEButtonStatus<source/stack/low_level/end_effector_interface:EEButtonStatus (Message)>`
      -  Publishes state of Custom Button
   *  -  /niryo_robot_hardware_interface/end_effector_interface/_digital_out_publisher
      -  :ref:`EEIOState<source/stack/low_level/end_effector_interface:EEIOState (Message)>`
      -  Publishes state of IO Digital

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
- :doc:`common`

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


.. |namespace_cpp| replace:: end_effector_interface
.. |namespace| replace:: /end_effector_interface/
.. |namespace_emphasize| replace:: ``/end_effector_interface/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/end_effector_interface
