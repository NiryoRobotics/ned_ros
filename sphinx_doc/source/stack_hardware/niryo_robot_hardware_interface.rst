Niryo_robot_hardware_interface
=======================================

| This package handles packages related to the robot's hardware.
| It launches hardware interface nodes, motors communication and driver.  

.. figure:: ../../images/ros/hardware_stack_nodes.png
   :alt: hardware packages organization
   :height: 500px
   :align: center

   Global overview of hardware stack packages organization.

Hardware interface node
--------------------------
The ROS Node is made to launch hardware interface and communication:
 - Conveyor Belt interface
 - Joints interface
 - Tools interface
 - Fake interface
 - Dynamixel driver
 - Stepper driver

The namespace used is: |namespace_emphasize|.

Parameters - hardware interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Hardware Interface's Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``publish_hw_status_frequency``
      -  | Publishes rate for hardware status.
         | Default : '2.0'
   *  -  ``publish_software_version_frequency``
      -  | Publishes rate for software status.
         | Default : '2.0'

Published topics - hardware interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Hardware Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``hardware_status``
      -  :ref:`niryo_robot_msgs/HardwareStatus<source/ros/niryo_robot_msgs:HardwareStatus>`
      -  Motors, bus, joints and CPU status
   *  -  ``software_version``
      -  :ref:`niryo_robot_msgs/SoftwareVersion<source/ros/niryo_robot_msgs:SoftwareVersion>`
      -  Raspberry and stepper software version

Services - hardware interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Hardware Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``launch_motors_report``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Starts motors report
   *  -  ``reboot_motors``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Reboots motors
   *  -  ``stop_motors_report``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Stops motors report

Dependencies - hardware interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :doc:`tools_interface`
- :doc:`joints_interface`
- :doc:`conveyor_interface`
- :doc:`cpu_interface`
- :doc:`fake_interface`
- :doc:`../ros/niryo_robot_msgs`

.. |namespace_emphasize| replace:: ``/niryo_robot_hardware_interface/``
