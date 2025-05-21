Niryo Robot Hardware Interface
##############################

This package is in charge for managing drivers and required hardware interfaces of the robot.
It launches hardware interface nodes, motors communication and drivers.  

.. figure:: /.static/images/hardware_interface.png
   :alt: Hardware interfaces
   :height: 300px
   :align: center

   Global overview of hardware stack packages organization.

The hardware interface node instantiates all the hardware interfaces we need to have a fully functional robot.
The hardware interfaces use the TTL driver in order to communicate with all the devices (motors, end effector panel, accessories, ...) on the robot.

Among those interfaces we have:
 - Conveyor Interface
 - Joints Interface
 - Tools Interface
 - Cpu Interface
 - End Effector Panel Interface 
 - TTL Driver

It belongs to the ROS namespace: |namespace_emphasize|.

Hardware status
***************

The robot exposes the status of its hardware on the topic ``/niryo_robot_hardware_interface/hardware_status``.
You can check the robot's hardware current status by using the following command in a terminal launched on the robot:

.. code-block:: bash

    rostopic echo /niryo_robot_hardware_interface/hardware_status

Message definition
------------------

.. literalinclude:: /../niryo_robot_msgs/msg/HardwareStatus.msg
    :language: python

Package Documentation
*********************

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/niryo_robot_hardware_interface
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_hardware_stack/niryo_robot_hardware_interface/launch/niryo_robot_hardware_interface.launch

.. |namespace_emphasize| replace:: ``/niryo_robot_hardware_interface``