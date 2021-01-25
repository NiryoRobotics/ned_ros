Niryo Robot Conveyor Interface Package
===========================================

| This package handles Niryo's conveyor
| You can control two conveyors

Conveyor Interface Node
--------------------------
The ROS Node is made to :
 - Initialize conveyor motor parameters
 - Set and control conveyors
 - Publish conveyors state

The namespace used is : |namespace_emphasize|

Parameters - Conveyor Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``publish_frequency``
      -  | Publish rate for conveyors state.
         | Default : '2.0'

Published Topics - Conveyor Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Conveyor Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``feedback``
      -  :ref:`ConveyorFeedbackArray<ConveyorFeedbackArray (Message)>`
      -  Conveyors states

Services - Conveyor Interface
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
      -  :ref:`ControlConveyor<ControlConveyor (Service)>`
      -  Send a command to the desired conveyor
   *  -  ``ping_and_set_conveyor``
      -  :ref:`SetConveyor<SetConveyor (Service)>`
      -  Scan and set a new conveyor

Dependencies - Conveyor Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :ref:`stepper_driver <Niryo Robot Stepper Driver Package>`

Services & Messages files - Conveyor Interface
----------------------------------------------

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
