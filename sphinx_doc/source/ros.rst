ROS Stack documentation
=========================================

.. figure:: ../images/ros_logo.png
   :alt: ROS Logo
   :width: 300px
   :align: center

   ROS Logo

ROS (Robot Operating System) is an Open-Source Robotic Framework which
allows to ease robot software development. The framework is used
in almost each part of Ned software.

The high-level packages (motion planner, vision, ...) are coded in Python to give
good readability whereas communication with Hardware is developed in C++ to ensure speed.

.. note::
   To learn more about ROS, go on `Official ROS Wiki <http://wiki.ros.org/>`_.

In this section, you will have access to all information about each Niryo Robot's ROS packages.

.. toctree::
   :maxdepth: 2
   :caption: ROS Documentation Sections

   ros/niryo_robot_bringup
   ros/niryo_robot_arm_commander
   ros/niryo_robot_description
   ros/niryo_robot_gazebo
   ros/niryo_robot_msgs
   ros/niryo_robot_modbus
   ros/niryo_robot_poses_handlers
   ros/niryo_robot_programs_manager
   ros/niryo_robot_rpi
   ros/niryo_robot_tools_commander
   ros/niryo_robot_unit_tests
   ros/niryo_robot_user_interface
   ros/niryo_robot_vision
