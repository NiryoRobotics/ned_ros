*******************************************
Use your Niryo Robot
*******************************************

.. list-table::
   :header-rows: 0
   :widths: auto
   :align: center

   *  -  .. image:: ../images/niryo_one_front.jpg
                  :alt: One
                  :width: 300px
                  :align: center
      -  .. image:: ../images/niryo_ned_front.jpg
                  :alt: Ned
                  :width: 300px
                  :align: center
      -  .. image:: ../images/niryo_ned2_front.png
                  :alt: Ned
                  :width: 300px
                  :align: center

Every Niryo Robot is usable as it is when first switched on, with Niryo Studio for instance.
However this robot can be used in many more ways if you want to go deeper into its understanding.

In this tutorial, we will explain how the robot is setup and the different options you have to control it.

Connecting to the Robot
======================================
You can connect to your robot in multiple ways (Ethernet direct, Wi-Fi Hotspot, LAN).

You can find more information on how to connect to your robot `here <https://docs.niryo.com/product/niryo-studio/source/connection.html>`_.

Once your robot is accessible from your computer, you can access it through three ways:

* Via Niryo Studio
  
  Niryo Studio provides you with all the tools you need to control the robot.
  Please refer to the :doc_niryo_studio:`Niryo Studio documentation <>` for more information.
  
* Via ROS Multimachine.
  
  ROS implements a way to communicate between nodes launched on different machines.
  By indicating your computer where the Niryo Robot ROS Master Node is, you can communicate to any ROS Node as 
  if you were directly connected on the robot. 
  See the tutorial on the :ros_multimachine:`ROS wiki <>` for more information.

* Via ssh (**for advanced users only**). 
  
  Port 22 is open without restriction. The default user is "niryo" and its password is "robotics".

Robot setup
=========================================

To help you understand how the OS is setup in the robot, we provide you with some insights of it.

.. attention::
   This document is not intended to explain how to completely install a robot from an empty SD card.
   It is only intended to give you clues on its architecture.
   Some of the installation steps are refered in :ref:`source/installation/ubuntu_18:Ubuntu 18 Installation`
   in case you would like to reinstall some part of it (catkin_ws for example).

System setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The robot is running on top of an Ubuntu server 18.04.5 for ARM customized to work on a Raspberry Pi 4B.

It comprises all the following elements :
   - ROS melodic and its requirements
   - Sound driver
   - Led ring driver
   - Robot System services (connectivity, databases, flask server)
   - Basic development tools (compilation, editing tools)

We took care to update and upgrade the system before sending it to you

.. attention::
   We can't ensure that the stability of the system will be kept if you try to update your system by yourself (using apt).
   We strongly advise you not to do so. 

Home setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The system has been configured with a default user "niryo".
The core of the robot program is installed in the home directory of niryo user */home/niryo*.

In this directory, you have:

- catkin_ws : contains the source code and the compiled binary for the Niryo ROS Stack
- firmware_updater : updater for the steppers and the End Effector
- niryo_robot_saved_files : set of files saved on the robot, used by Niryo Studio
- system_software : configuration files for system wide functions

Services and daemons
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Two services are used on the robot: 

- niryo_system_software : It launches the flask server for API communication with the robot

- niryo_robot_ros : It launches the stack via */opt/start_ros.sh* script at startup. 

File */opt/start_ros.sh* on the ned2 robot :

.. code::

   source ~/.bashrc
   source /home/niryo/catkin_ws/install/release/ned2/setup.bash && roslaunch niryo_robot_bringup niryo_ned2_robot.launch&


If you want to start, stop or disable one of those services, please refer to the `dedicated manual <https://manpages.ubuntu.com/manpages/bionic/man8/service.8.html>`_.


Starting the robot manually (for advanced users only)
========================================================
Before continuing, be sure you know what you are doing. 

You will need to have a ssh access setup to continue.


Stopping the service
^^^^^^^^^^^^^^^^^^^^^^^^^^
First you will need to stop the Niryo ROS Stack that is automatically started when the robot boots up.
Use the system linux command to do so:

.. code::

   sudo service niryo_robot_ros stop

Starting the robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^
To start the robot, launch the following commands in a ssh terminal:

For Ned

.. code::

   source /home/niryo/catkin_ws/install/release/ned/setup.bash
   roslaunch niryo_robot_bringup niryo_ned_robot.launch

For Ned2

.. code::

   source /home/niryo/catkin_ws/install/release/ned2/setup.bash
   roslaunch niryo_robot_bringup niryo_ned2_robot.launch

.. list-table:: Robot launch options
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Default Value 
      - Description
   *  - log_level
      - INFO
      - Log level to display for ROS loggers
   *  - ttl_enabled
      - true
      - | Enable or disable the TTL bus usage. 
        | This feature is used for debug mainly and can lead to an unstable stack.
   *  - can_enabled
      - true
      - | Enable or disable the CAN bus usage. 
        | This feature is used for debug mainly and can lead to an unstable stack.
   *  - debug
      - false
      - Launch in debug mode. For development and debug only.
   *  - timed
      - true
      - | To launch the node using timed_roslaunch. 
        | If enabled, will first launch sound and light nodes to have a better user experience. 
        | If disabled, the node is directly launched
  
Changing the log level
^^^^^^^^^^^^^^^^^^^^^^^^^^
Before launching the robot, you can change the configuration file for the ROS Logger in order to change the log level displayed on the terminal.
This file is located in */home/niryo/catkin_ws/src/niryo_robot_bringup/config/niryo_robot_trace.conf*.

It defines the logs levels for all cpp packages, based on log4cxx configuration file syntax.
Please see documentation of :rosconsole:`rosconsole <>` or :log4cxx:`log4cxx <>` for more information.

By default, the level is set to INFO, you can change this value if you want more logs.

.. code::

   # Set the default ros output to warning and higher
   log4j.logger.ros=INFO

.. attention::
   DEBUG level is very verbose, you can deteriorate the performances of your robot by doing so.

You can also choose to change only the log level of a specific cpp package by uncommenting one of the following lines and 
optionally change the associated log level.

.. code::

   #log4j.logger.ros.can_driver = DEBUG
   log4j.logger.ros.common = DEBUG
   log4j.logger.ros.conveyor_interface = ERROR

