Tools Interface
###############

| This package handles Niryo's tools.

Tools interface node (Only for development and debugging)
*********************************************************

The ROS Node is used to:
 - initialize the Tool Interface.
 - start ROS stuffs like services, topics.

Tools Interface Core
********************

It is instantiated by the :doc:`niryo_robot_hardware_interface` node.

The node:
 - initializes the Tool Interface.
 - provides services for setting and controlling tools.
 - publishes tools states.

It belongs to the ROS namespace: |namespace_emphasize|.

Tools motors configuration
**************************

Niryo tools are equipped with dynamixels servomotors, therefore they can be configured following the dynamixel protocol.

Here is are the parameters for the motors of all the Niryo robot tools:

.. literalinclude:: /../niryo_robot_hardware_stack/tools_interface/config/ned3pro/tools_params.yaml
   :language: yaml
   :linenos:

In this yaml file, each index for each list corresponds to a tool. Each tool has a name, an ID on the TTL bus, a type and a set of parameters.

Package Documentation
*********************

For more informations about ROS topics, services and parameters, check the :doc:`Niryo robot hardware interface documentation <niryo_robot_hardware_interface>`.

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/tools_interface
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_hardware_stack/tools_interface/launch/tools_interface_base.launch.xml
    :is_hardware_interface:
    :hardware_interface_node_namespace: /tools

.. |namespace_emphasize| replace:: ``/tools_interface``