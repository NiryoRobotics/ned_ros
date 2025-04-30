TTL Driver
##########

This package implements the protocol to communicate with the devices connected to the TTL bus of the robot (Motors, end effector panel, accessories, ...).

This package is based on the DXL SDK. It provides an interface to :wiki_ros:`the ROS package wiki <dynamixel_sdk>`.

TTL Driver Node (Only for development and debugging)
***************************************************************

This node:
 - initializes the TTL Interface.
 - gets the motor's configuration and add them to TTL Interface.

TTL Interface Core
******************

It is instantiated by the :doc:`niryo_robot_hardware_interface` node.

It has been conceived to:
 - Initialize the TTL Interface (Interface used by other packages) and physical bus with the configurations.
 - Add, remove and monitor devices.
 - Start getting data and sending data on the physical bus.
 - Start ROS stuffs like services, topics.

It belongs to the ROS namespace: |namespace_emphasize|.

Package Documentation
*********************

For more informations about ROS topics, services and parameters, check the :doc:`Niryo robot hardware interface documentation <niryo_robot_hardware_interface>`.

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/ttl_driver
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_hardware_stack/ttl_driver/launch/ttl_driver_base.launch.xml
    :is_hardware_interface:
    :hardware_interface_node_namespace: /ttl_driver

.. |namespace_emphasize| replace:: ``/ttl_driver``
