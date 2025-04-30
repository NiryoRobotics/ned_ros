Joints Interface
################

This package acts as an interface for the robot's joints controller.

It provides an interface to :wiki_ros:`the ROS control package <ros_control>`.

Joints interface node
*********************

It is instantiated by the :doc:`niryo_robot_hardware_interface` node.

The node:
 - interfaces robot's motors to the joint trajectory controller, from :wiki_ros:`the ROS control package <ros_control>`.
 - creates a controller manager, from :wiki_ros:`the ROS control package <ros_control>`, that provides the infrastructure to load, unload, start and stop controllers.
 - interfaces with motors calibration.
 - initializes motors parameters.

It belongs to the ROS namespace: |namespace_emphasize|.

Joints motors configuration
***************************

Here are the configurations for each robot joint motor.
They contain the name of the joint, its ID on the TTL bus and its type.

**For the Ned2 robot:**

.. literalinclude:: /../niryo_robot_hardware_stack/joints_interface/config/ned2/joints_params.yaml
   :language: yaml
   :linenos:

**For the Ned3pro robot:**

.. literalinclude:: /../niryo_robot_hardware_stack/joints_interface/config/ned3pro/joints_params.yaml
   :language: yaml
   :linenos:


The stepper motors profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Ned robots use stepper motors for the first 3 axis.
The velocity and acceleration profiles for the stepper motors are defined in the *steppers_params.yaml* config file.

**For the Ned2 robot:**

.. literalinclude:: /../niryo_robot_hardware_stack/joints_interface/config/ned2/steppers_params.yaml
   :language: yaml
   :linenos:

**For the Ned3pro robot:**

.. literalinclude:: /../niryo_robot_hardware_stack/joints_interface/config/ned3pro/steppers_params.yaml
   :language: yaml
   :linenos:

These parameters allows us to define this kind of velocity profile for the Ned robot's motors:

.. figure:: /.static/images/steppers_profiles.png
   :alt: TTL steppers velocity/acceleration profiles
   :width: 600px
   :align: center

The dynamixel motors parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Ned robots use dynamixels servomotors for the last 3 axis.
The parameters for the dynamixel motors are defined in the *dynamixels_params.yaml* config file.

**For the Ned2 robot:**

.. literalinclude:: /../niryo_robot_hardware_stack/joints_interface/config/ned2/dynamixels_params.yaml
   :language: yaml
   :linenos:

**For the Ned3pro robot:**

.. literalinclude:: /../niryo_robot_hardware_stack/joints_interface/config/ned3pro/dynamixels_params.yaml
   :language: yaml
   :linenos:

These parameters allows us to configure some features, like the PID control gains, for the Ned robot's dynamixel motors following `the dynamixel control table <https://emanual.robotis.com/docs/en/dxl/x/xh430-w210/#position-pid-gain80-82-84>`_ of the current motor.

Package Documentation
*********************

For more informations about ROS topics, services and parameters, check the :doc:`Niryo robot hardware interface documentation <niryo_robot_hardware_interface>`.

.. rosdoc:: /niryo_robot_hardware_interface
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_hardware_stack/joints_interface
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_hardware_stack/joints_interface/launch/joints_interface_base.launch.xml
    :is_hardware_interface:
    :hardware_interface_node_namespace: /joints_interface

.. |namespace_emphasize| replace:: ``/joints_interface``

