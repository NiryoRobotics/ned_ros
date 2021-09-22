Conveyor_interface
===========================================

| This package handles Niryo's Conveyor Belt.
| You can control two Conveyors Belt.

Conveyor Belt interface node
-----------------------------
The ROS Node is made to:
 - Initialize Conveyor Belt motor parameters.
 - Set and control Conveyors Belt.
 - Publish Conveyors Belt state.

The namespace used is: |namespace_emphasize|

Parameters - Conveyor Belt interface
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

Services - Conveyor Belt interface
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
      -  Scans and sets a new Conveyor Belt

Dependencies - Conveyor Belt interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :doc:`stepper_driver <stepper_driver>`

Services & messages files - Conveyor Belt interface
------------------------------------------------------

ControlConveyor (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/conveyor_interface/srv/ControlConveyor.srv
   :language: rostype

SetConveyor (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/conveyor_interface/srv/SetConveyor.srv
   :language: rostype

ConveyorFeedbackArray (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/conveyor_interface/msg/ConveyorFeedbackArray.msg
   :language: rostype

ConveyorFeedback (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_hardware_stack/conveyor_interface/msg/ConveyorFeedback.msg
   :language: rostype

.. |namespace_emphasize| replace:: ``/niryo_robot/conveyor/``
