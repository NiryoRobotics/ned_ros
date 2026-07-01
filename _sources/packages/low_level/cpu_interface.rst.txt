CPU Interface
#############

| This package provides an interface for CPU temperature monitoring.

CPU Interface Node (Only for development and debugging) 
*******************************************************

This ROS Node is used to run the CPU interface in standalone.

CPU Interface Core
******************

It is instantiated by the :doc:`niryo_robot_hardware_interface` node.

It is responsible to monitor CPU temperature of the Raspberry Pi and automatically shutdown the Raspberry Pi if it reaches a critical threshold.
Two thresholds can be defined via parameters: a warning threshold and a shutdown threshold.

The CPU temperature is read from the Ubuntu system file */sys/class/thermal/thermal_zone0/temp*.

In simulation, the CPU temperature of the computer running the simulation is used, but the threshold are deactivated (no shutdown in case of high temperature).

It belongs to the ROS namespace: |namespace_emphasize|.

Configuration
*************

The configuration for the CPU interface node is defined in the *default.yaml* config file.

.. literalinclude:: /../niryo_robot_hardware_stack/cpu_interface/config/default.yaml
   :language: yaml
   :linenos:

Package Documentation
*********************

For more informations about ROS topics, services and parameters, check the :doc:`Niryo robot hardware interface documentation <niryo_robot_hardware_interface>`.

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/cpu_interface
    :is_hardware_interface:
    :hardware_interface_node_namespace: /cpu_interface

.. |namespace_emphasize| replace:: ``/cpu_interface``