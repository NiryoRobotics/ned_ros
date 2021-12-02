Conveyor Interface
===========================================

| This package handles Niryo's Conveyor Belt.
| Actually, you can control two Conveyors Belt in the same time.

Conveyor Belt interface node (For development and debugging) 
-----------------------------
The ROS Node is made to:
 - Get low level driver compatible with the robot using.
 - Initialize Conveyor Belt Interface

Conveyor Belt Interface
----------------------------
It is integrated to :doc:`niryo_robot_hardware_interface` package

 - Get low level driver
 - Initialize Conveyor Belt motor parameters.
 - Wait the request from service to set and Control Conveyors Belt or to remove Conveyors Belt.
 - Publish Conveyor Belt states.

The namespace used is: |namespace_emphasize|

Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Belt Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``publish_frequency``
      -  | Publishes rate for conveyors state.
         | Default: '2.0'

   *  -  ``type``
      -  | Type of the motor using.
         | Default: 'Stepper'

   *  -  ``protocol``
      -  | Protocol of the communication.
         | It can be 'CAN' or 'TTL'

   *  -  ``default_id``
      -  | Default id of Conveyor Belt before the connection.

   *  -  ``Pool_id_list``
      -  | Id of Conveyor Belt after the connection.

   *  -  ``Direction``
      -  | The direction of the Conveyor Belt.

   *  -  ``max_effort``
         (CAN Only)
      -  | Max effort using by stepper
         | Default: '90'

   *  -  ``micro_steps``
         (CAN only)
      -  | Micro steps used by Stepper
         | Default: '8'

Published topics - Conveyor Belt interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Belt Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``feedback``
      -  :ref:`ConveyorFeedbackArray<source/stack_hardware/conveyor_interface:ConveyorFeedbackArray (Message)>`
      -  Conveyors Belt states

Services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Belt Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``control_conveyor``
      -  :ref:`ControlConveyor<source/stack_hardware/conveyor_interface:ControlConveyor (Service)>`
      -  Sends a command to the desired Conveyor Belt
   *  -  ``ping_and_set_conveyor``
      -  :ref:`SetConveyor<source/stack_hardware/conveyor_interface:SetConveyor (Service)>`
      -  Scans and sets a new Conveyor Belt or remove a Conveyor Belt connected

Dependencies - Conveyor Belt interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :doc:`can_driver`
- :doc:`ttl_driver`

Services & messages files - Conveyor Belt interface
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

.. |namespace_emphasize| replace:: ``/niryo_robot/conveyor/``
