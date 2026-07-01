Common
######

| The Common package defines the common software components of the low level software implementation. It is split into a model part and a utility part:
| - The **model** subpackage defines the model tree needed to keep a virtual state of the robot up to date at any time.
| - The **util** subpackage defines cpp interfaces and useful functions

Model
*****
The model subpackage contains:

States
^^^^^^
Classes representing the virtual state of each hardware component at any moment. 
The hierarchy allows powerful polymorphism so that we can interpret each component differently based on the information we need to obtain. 

.. figure:: /.static/images/classcommon_1_1model_1_1AbstractHardwareState__inherit__graph.png
   :alt: Abstract Hardware State inheritance graph 
   :height: 400px
   :align: center

   Abstract Hardware State inheritance graph 

Enums
^^^^^
They are enhanced enums to keep trace of various enumeration and be able to have useful utilities attached to them (like conversion to string).

.. figure:: /.static/images/classcommon_1_1model_1_1HardwareTypeEnum__inherit__graph.png
   :alt: Hardware Type Enum inheritance graph
   :height: 100px
   :align: center

   Hardware Type Enum inheritance graph

Commands
^^^^^^^^
Classes representing single and synchronize commands, for steppers and dynamixels. They are used to queue commands in the ttl_driver packages.

.. |picSingle| image:: /.static/images/classcommon_1_1model_1_1ISingleMotorCmd__inherit__graph.png
   :alt: ISingleMotorCmd inheritance graph
   :width: 200px


.. |picSync| image:: /.static/images/classcommon_1_1model_1_1ISynchronizeMotorCmd__inherit__graph.png
   :alt: ISynchronizeMotorCmd inheritance graph
   :width: 200px
   
.. table:: Commands graphs
   :widths: auto
   :align: center

   ============  ===========
   |picSingle|    |picSync|
   ============  ===========
    Single Cmd    Sync Cmd
   ============  ===========

Each type of command is an alias to specified versions of two base template classes: AbstractSynchronizeMotorCmd and AbstractSingleMotorCmd

Util
****
The util subpackage contains:

* Cpp interfaces, used globally in the stack for polymorphism
* Utility functions usable globally in the stack