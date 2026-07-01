Common interfaces
#################


The Niryo robot msgs package
****************************

This package is used to define custom shared Niryo ROS interfaces.

.. rosdoc_standalone_interface_package:: ../niryo_robot_msgs

The ``/niryo_robot`` namespace
******************************

In Ned robots, most of the ROS interfaces and parameters are bound to a specific package and, therefore, to a specific namespace.
However, some of them are considered `global` for the robot and take the namespace ``/niryo_robot``.

Here is an overview of the common interfaces available in the robot:

.. rosdoc:: /niryo_robot/
    :is_global_namespace: True
    :description_file: packages/descriptions.yaml


