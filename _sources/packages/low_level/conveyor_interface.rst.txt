Conveyor Interface
##################

| This package provides and interface Niryo's Conveyors. 

| It allows you to control up to two Conveyors at the same time via the TTL bus.


Conveyor Interface node (Only development and debugging) 
********************************************************

This ROS Node is used to:
 - Expose ROS interfaces to control or get states from the conveyor.
 - Initialize the Conveyor Interface.

Conveyor Interface core
***********************

It is instantiated by the :doc:`niryo_robot_hardware_interface` node.

The node:
   - interfaces with the TTL driver.
   - initialize conveyor motors parameters.
   - handles the requests from services to set, control or remove the conveyors.
   - publishes conveyor states.

It belongs to the ROS namespace: |namespace_emphasize|.

Package Documentation
*********************

For more informations about ROS topics, services and parameters, check the :doc:`Niryo robot hardware interface documentation <niryo_robot_hardware_interface>`.

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/conveyor_interface
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_hardware_stack/conveyor_interface/launch/conveyor_interface_base.launch.xml
    :is_hardware_interface:
    :hardware_interface_node_namespace: /conveyor

.. |namespace_emphasize| replace:: ``/niryo_robot/conveyor``