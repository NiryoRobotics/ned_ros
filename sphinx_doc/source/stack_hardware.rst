Hardware ROS Stack documentation
=========================================

.. figure:: ../images/ros_logo.png
   :alt: ROS Logo
   :width: 300px
   :align: center

   ROS Logo

ROS (Robot Operating System) is an Open-Source Robotic Framework which
allows to ease robot software development. The framework is used
in almost each part of Ned's software.

The high-level packages (motion planner, vision, ...) are coded in Python to give
good readability whereas communication with Hardware is developed in C++ to ensure speed.

.. note::
   To learn more about ROS, go on `Official ROS Wiki <http://wiki.ros.org/>`_.

In this section, you will have access to all information about each Niryo robot's ROS hardware stack packages.

.. toctree::
   :maxdepth: 2
   :caption: ROS Documentation Sections

   stack_hardware/niryo_robot_hardware_interface
   stack_hardware/joints_interface
   stack_hardware/conveyor_interface
   stack_hardware/tools_interface
   stack_hardware/cpu_interface
   stack_hardware/fake_interface
   stack_hardware/dynamixel_driver
   stack_hardware/stepper_driver
   stack_hardware/niryo_robot_debug