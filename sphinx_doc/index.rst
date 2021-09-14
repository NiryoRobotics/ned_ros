.. Ned Doc documentation master file, created by
   sphinx-quickstart on Thu Jul 23 10:15:29 2020.

#####################################
Ned ROS documentation
#####################################

| This documentation contains everything you need to understand Ned's
 functioning & how to control it through ROS.
| It is made as well for users who are
 using the "physic" robot as those who want to use a virtual version.

.. list-table::
   :header-rows: 0
   :widths: auto
   :align: center

   *  -  .. image:: images/ros_logo.png
                  :alt: ROS Logo
                  :width: 300px
                  :align: center
      -  .. image:: images/niryo_ned_front.jpg
                     :alt: Ned
                     :width: 300px
                     :align: center


**************
Preamble
**************

Before diving into the software documentation, you can learn more about the robot development
in the :doc:`Overwiew <source/overview>` section.

Also, if you want to use a simulated version of Ned, browse the :doc:`Simulation <source/simulation>` section.

****************************
Ned Control via ROS
****************************

Ned is fully based on ROS.

ROS Direct control
---------------------------------

.. important::
   To control the robot directly with ROS, you will need either to be connected in SSH
   to the physic robot, or to use the simulation.

ROS is the most direct way to control the robot. It allows:

- to send command via the terminal in order to call services, trigger action, ...
- to write a entire Python/C++ node in a script to realize a full process.

See :doc:`ROS <source/ros>` section to see all Topic & Services available.

Python ROS Wrapper
---------------------------------
.. important::
   To use Python ROS Wrapper, you will need either to be connected in SSH
   to the physic robot, or to use the simulation.

The Python ROS Wrapper is built on top of ROS to allow a faster development than ROS.
Programs are run directly on the robot which allows to trigger them with the robot's button
once a computer is no longer needed.

See :doc:`Python ROS Wrapper <source/ros_wrapper>` to see which functions are accessible and examples on
how to use them.

More ways
---------------------------------

Other methods are available to control the robot allowing the user
to code and run programs outside his terminal.

Learn more on this :doc:`section <source/more>`.


.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: Introduction

   source/overview
   source/simulation

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: Packages

   source/ros
   source/stack_hardware
   source/ros_wrapper

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: To go further...

   source/tcp_server
   source/modbus_tcp_server
   source/more
