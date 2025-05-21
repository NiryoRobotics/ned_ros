End Effector Interface
######################

| This package handles the End Effector Panel of a robot.
| It provides services and topics specific to the End Effector Panel.

End Effector Interface node (Only for development and debugging)
****************************************************************

The ROS Node is used to:
 - initialize the End Effector Interface.
 - Expose ROS interfaces to control or get states from the end effector panel.

End Effector Interface Core
***************************

It is instantiated by the :doc:`niryo_robot_hardware_interface` node.

The node:
 - interfaces with the TTL Driver.
 - initializes End Effector parameters.
 - retrieves End Effector data from TTL driver.
 - publishes the status of buttons.
 - publishes the collision detection status.
 - starts service for the panel IO States.

It belongs to the ROS namespace: |namespace_emphasize|.

Package Documentation
*********************

For more informations about ROS topics, services and parameters, check the :doc:`Niryo robot hardware interface documentation <niryo_robot_hardware_interface>`.

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/end_effector_interface
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_hardware_stack/end_effector_interface/launch/end_effector_interface_base.launch.xml
    :is_hardware_interface:
    :hardware_interface_node_namespace: /end_effector_interface

.. |namespace_emphasize| replace:: ``/end_effector_interface``

