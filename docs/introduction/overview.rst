ROS Stack overview
==================

Ned robots are based on Raspberry & ROS. They use ROS to
make the interface between Hardware and high-level bindings.

On the following figure, you can see a global overview of Ned robot's software architecture.
It will help you understand where are placed each part of the software.


.. figure:: /.static/images/ROS_stack_overview.png
   :alt: Ned Robotcs software architecture
   :width: 1200px
   :align: center

   Ned Robots software architecture

The TCP server exposed in this schematic follows our own protocole, see :doc:`Use Ned robot's TCP server <../to_go_further/tcp_server>` for more informations.