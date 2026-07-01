Overview
########

Ned2 is a robot based on Raspberry, Arduino & ROS. It uses ROS to
make the interface between Hardware and high-level bindings.

On the following figure, you can see a global overview of the Niryo's robot software
in order to understand where are placed each part of the software.


.. figure:: ../.static/images/ROS_stack_overview.png
   :alt: Ned2 software architecture
   :figwidth: 100%
   :align: center

   Ned2 software architecture

The TCP server exposed in this schematic follows our own protocole, see :doc:`Use Ned robot's TCP server <../to_go_further/tcp_server>` for more informations.

ROS (Robot Operating System) is an Open-Source Robotic Framework which
allows to ease robot software development. The framework is used
in almost each part of Ned2's software.

The Stack is split into two parts:

- The :doc:`high_level` (motion planner, vision, ...), developed in Python to give good readability.
- The :doc:`low_level` (drivers, hardware management, ...), developed in C++ to ensure real time capabilities.

.. note::
   To learn more about ROS, go on `Official ROS Wiki <http://wiki.ros.org/>`_.