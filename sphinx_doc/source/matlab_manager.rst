Matlab manager
========================================

This package allow to receive :ref:`RobotCommand<RobotCommand (message)>` from Matlab and move the robot accordingly.

Matlab manager node
--------------------------
The ROS Node is made of one topic that enables to receive goals from Matlab.

Dependencies - Matlab manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- :ref:`niryo_robot_commander <Niryo Robot Commander Package>`


Services files - Matlab manager
-----------------------------------

MatlabMoveResult (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../niryo_robot_user_interface/msg/MatlabMoveResult.msg
   :language: rostype

.. |namespace| replace:: /niryo_robot_matlab/
.. |namespace_emphasize| replace:: ``/niryo_robot_matlab/``
.. |package_path| replace:: ../../niryo_robot_user_interface
