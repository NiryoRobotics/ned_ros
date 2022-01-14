CPU Interface
=================================

| This package provides an interface for CPU temperature monitoring.



CPU Interface Node (For development and debugging purpose only) 
-----------------------------------------------------------------
This ROS Node has been conceived to launch the CPU interface in an isolated way.

CPU Interface Core
----------------------------
It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been made to monitor CPU temperature of the Raspberry Pi and automatically shutdown the Raspberry Pi if it reaches a critical threshold.
Two thresholds can be defined via parameters: a warning threshold and a shutdown threshold.

The CPU temperature is read from the Ubuntu system file */sys/class/thermal/thermal_zone0/temp*.

In simulation, the CPU temperature of the computer running the simulation is used, but the threshold are deactivated (no shutdown in case of high temperature).

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: CPU Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``read_rpi_diagnostics_frequency``
      -  | Publishes rate for CPU temperature
         | Default: '0.25'
   *  -  ``temperature_warn_threshold``
      -  | CPU temperature [celsius] threshold before a warn message
         | Default: '75'
   *  -  ``temperature_shutdown_threshold``
      -  | CPU temperature [celsius] threshold before shutdown the robot
         | Default: '85'


Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- :doc:`common`


Services, Topics and Messages
-------------------------------------------------
None


.. |namespace_cpp| replace:: cpu_interface
.. |namespace| replace:: /cpu_interface/
.. |namespace_emphasize| replace:: ``/cpu_interface/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/cpu_interface

