Niryo robot hardware interface package
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
      -  | Publish rate for hardware status.
         | Default : '2.0'
   *  -  ``publish_software_version_frequency``
      -  | Publish rate for software status.
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
      -  :ref:`niryo_robot_msgs/HardwareStatus<HardwareStatus>`
      -  Motors, bus, joints and CPU status
   *  -  ``software_version``
      -  :ref:`niryo_robot_msgs/SoftwareVersion<SoftwareVersion>`
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
      -  :ref:`Trigger`
      -  Start motors report
   *  -  ``reboot_motors``
      -  :ref:`Trigger`
      -  Reboot motors
   *  -  ``stop_motors_report``
      -  :ref:`Trigger`
      -  Stop motors report

Dependencies - hardware interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :ref:`tools_interface <Niryo Robot Tools Interface Package>`
- :ref:`joints_interface <Niryo Robot Joints Interface Package>`
- :ref:`conveyor_interface <Niryo robot Conveyor Belt interface package>`
- :ref:`cpu_interface <Niryo Robot CPU Interface Package>`   
- :ref:`fake_interface <Niryo Robot Fake Interface Package>`     
- :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`

.. |namespace_emphasize| replace:: ``/niryo_robot_hardware_interface/``
