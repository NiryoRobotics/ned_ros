********************
Use your Niryo Robot
********************

Every Ned robot is ready to use right out of the box, for example, with  `Niryo Studio <https://niryo.com/fr/niryostudio/>`_.
However, the robot offers many more possibilities if you want to delve deeper into its functionalities.

In this tutorial, we will explain how the robot is set up and the various options available for controlling it.

Connecting to the Robot
=======================
You can connect to your robot in multiple ways, including direct Ethernet, Wi-Fi hotspot, or LAN.

For more information on how to connect to your robot, refer to :doc_niryo_studio:`this guide <connecting-to-your-robot>`.

Once your robot is accessible from your computer, you can interact with it in three ways:

* Via ROS Multimachine.
  
  ROS implements a method for communication between nodes running on different machines.
  By specifying the location of the Niryo Robot ROS Master Node to your computer, you can communicate with any ROS node as if you were directly connected to the robot.
  See the tutorial on the :ros_multimachine:`ROS wiki <>` for more information.

* Via Niryo Studio
  
  Niryo Studio provides all the tools you need to control the robot.
  Please refer to the :doc_niryo_studio:`Niryo Studio documentation <>` for more information.

* Via ssh (**for advanced users only**). 
  
  Port 22 is open without restriction. The default user is **niryo** and its password is **robotics**.
  You can open a terminal and connect using ssh throught this command:
  
  .. code-block:: bash
   :caption: ssh with IP
   :linenos:

      ssh niryo@<robot_ip>

  If your robot is connected to a network with a DNS server you can replace the robot IP with its domain name:
  
  .. code-block:: bash
   :caption: ssh with DNS
   :linenos:

      ssh niryo@<robot_model>-<robot_id>.local

  Robot model can be either **ned2** or **ned3pro**.
  
  .. code-block:: bash
      :caption: ssh example
      :linenos:

      ssh niryo@ned3pro-7b-066-885.local


Robot setup
===========

To help you better understand the OS setup on the robot, we provide an overview of its architecture and key components.

.. attention::
   This document is not intended to provide a complete guide to installing the robot's OS from a blank SD card.  
   Instead, it aims to give you insights into the system's architecture.  
   Some installation steps are referenced in :doc:`../installation/install_for_ubuntu_20`  
   for situations where you may need to reinstall specific components (e.g., the `catkin_ws` workspace). 

System setup
^^^^^^^^^^^^

The robot operates on a customized Ubuntu Server 20.04 for ARM, tailored specifically for the Raspberry Pi 4B.

It includes the following components:
   - ROS noetic and its requirements
   - Sound driver
   - Led ring driver
   - Robot System services (connectivity, databases, flask server)
   - Basic development tools (compilation, editing tools)

We took care to update and upgrade the system before sending it to you.

.. attention::
   We cannot guarantee the system's stability if you attempt to update it manually (e.g., using apt).
   We strongly recommend against performing such updates.

Home setup
^^^^^^^^^^
The system is preconfigured with a default user account: **niryo**.
The core robot programs are installed in the niryo user's home directory: */home/niryo*.

In this directory, you have:

- **catkin_ws**: Contains the source code and compiled binaries for the Niryo ROS stack.
- **firmware_updater**: Responsible for updating the firmware of stepper motors, the end effector panel, and other embedded systems.
- **niryo_robot_saved_files**: A collection of files saved on the robot (programs, trajectories, ...), utilized by Niryo Studio.
- **system_software**: Holds configuration files for system-wide functionalities.

Services and daemons
^^^^^^^^^^^^^^^^^^^^
Two services are used on the robot: 

- niryo_system_software : It launches the flask server for API communication with the robot at startup.

- niryo_robot_ros : It launches the stack at startup.

If you want to start, stop or disable one of those services, please refer to the `dedicated manual <https://manpages.ubuntu.com/manpages/bionic/man8/service.8.html>`_.

Starting the robot manually (for advanced users only)
=====================================================
Before continuing, be sure you know what you are doing. 

You will need to have a ssh access setup to continue.


Stopping the service
^^^^^^^^^^^^^^^^^^^^
First, you need to stop the Niryo ROS Stack, which starts automatically when the robot boots up.
Use this system linux command to do so:

.. code:: bash

   sudo service niryo_robot_ros stop

Starting the robot
^^^^^^^^^^^^^^^^^^
To start the robot, launch the following commands in a ssh terminal:

Replace <robot_model> with either **ned2** or **ned3pro**, depending on your robot.

.. code:: bash

   source ~/setup.bash
   roslaunch niryo_robot_bringup niryo_<robot_model>_robot.launch

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
   *  - database
      - true
      - | Launch database related ROS nodes.
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
^^^^^^^^^^^^^^^^^^^^^^
Before launching the robot, you can change the configuration file for the ROS Logger in order to change the log level displayed on the terminal.
This file is located in */home/niryo/catkin_ws/src/niryo_robot_bringup/config/niryo_robot_trace.conf*.

It defines the logs levels for all cpp packages, based on log4cxx configuration file syntax.
Please see documentation of `rosconsole <http://wiki.ros.org/rosconsole>`_ or `log4cxx <https://logging.apache.org/log4cxx/latest_stable/index.html>`_ for more information.

By default, the level is set to INFO, you can change this value if you want more logs.

.. code:: yaml

   # Set the default ros output to warning and higher
   log4j.logger.ros=INFO

.. attention::
   DEBUG level is very verbose, you can deteriorate the performances of your robot by doing so.

You can also choose to change only the log level of a specific cpp package by uncommenting one of the following lines and 
optionally change the associated log level.

.. code:: yaml

   #log4j.logger.ros.can_driver = DEBUG
   log4j.logger.ros.common = DEBUG
   log4j.logger.ros.conveyor_interface = ERROR

