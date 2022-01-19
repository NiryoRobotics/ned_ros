Conveyor Interface
===========================================

| This package handles Niryo's Conveyors. 


| It allows you to control up to two Conveyors at the same time.

Two version of the conveyor exist: The Conveyor Belt, communicating via a CAN bus, and the Conveyor Belt (V2), communicating via a TTL bus.
Both of them are directly compatible for the Ned and One. For Ned2, you will need to change the stepper card of the CAN Conveyor Belt 
to be able to use it on a TTL port (there is no CAN port on Ned2).

Conveyor Interface node (For development and debugging purpose only) 
------------------------------------------------------------------------
This ROS Node has been conceived to:
 - Use the correct low level driver according to the hardware version of the robot.
 - Initialize the Conveyor Interface.

Conveyor Interface core
----------------------------
It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been conceived to:
   - Interface itself with low level drivers (CAN or TTL for Ned and Niryo One, TTL only for Ned2)
   - Initialize conveyor motors parameters.
   - Handle the requests from services to set, control or remove the conveyors.
   - Publish conveyor states.

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``publish_frequency``
      -  | Publishing rate for conveyors state.
         | Default: '2.0'

   *  -  ``type``
      -  | Type of the motor used.
         | Default: 'Stepper'

   *  -  ``protocol``
      -  | Protocol of the communication.
         | It can be 'CAN' (for Ned or One) or 'TTL' (for Ned or One or Ned 2)

   *  -  ``default_id``
      -  | Default id of the conveyor before the connection.

   *  -  ``Pool_id_list``
      -  | Id of the conveyor after the connection.

   *  -  ``Direction``
      -  | Direction of the conveyor.

   *  -  ``max_effort``
         (CAN Only)
      -  | Max effort used by the steppers
         | Default: '90'

   *  -  ``micro_steps``
         (CAN only)
      -  | Micro steps used by the Steppers
         | Default: '8'

Published topics - Conveyor interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``feedback``
      -  :ref:`ConveyorFeedbackArray<source/stack/low_level/conveyor_interface:ConveyorFeedbackArray (Message)>`
      -  Conveyors states

Services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``control_conveyor``
      -  :ref:`ControlConveyor<source/stack/low_level/conveyor_interface:ControlConveyor (Service)>`
      -  Sends a command to the desired Conveyor
   *  -  ``ping_and_set_conveyor``
      -  :ref:`SetConveyor<source/stack/low_level/conveyor_interface:SetConveyor (Service)>`
      -  Scans and sets a new Conveyor or removes a connected Conveyor

Dependencies - Conveyor interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :doc:`can_driver`
- :doc:`ttl_driver`

Services & messages files - Conveyor interface
------------------------------------------------------

ControlConveyor (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/conveyor_interface/srv/ControlConveyor.srv
   :language: rostype

SetConveyor (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/conveyor_interface/srv/SetConveyor.srv
   :language: rostype

ConveyorFeedbackArray (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/conveyor_interface/msg/ConveyorFeedbackArray.msg
   :language: rostype

ConveyorFeedback (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_hardware_stack/conveyor_interface/msg/ConveyorFeedback.msg
   :language: rostype


.. |namespace_cpp| replace:: conveyor_interface
.. |namespace| replace:: /niryo_robot/conveyor/
.. |namespace_emphasize| replace:: ``/niryo_robot/conveyor/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/conveyor_interface

