Niryo robot rpi
###############

This package deals with Raspberry Pi related stuff (Button, fans, I/O, leds, ...).

The package manages the following components:

- Physical top button: executes actions when the button is pressed.
- Digital I/O panel: gets commands and sends the current state of digital I/Os. Also controls tools like the Electromagnet.
- Analog I/O panel: gets commands and sends the current state of analog I/Os.
- End Effector I/O panel: gets commands and sends the current state of the digital I/Os of the end effector panel on Ned2. Also controls tools like the Electromagnet.
- Robot fans.
- Shutdown Manager: shutdown or reboot the Raspberry.
- ROS log: can remove all previous logs on start_up to prevent a lack of disk space in the long run (SD cards do not have infinite storage).

It belongs to the ROS namespace: |namespace_emphasize|.

.. |namespace_emphasize| replace:: ``/niryo_robot_rpi``

.. warning::

    This package should not be used when you are using Ned ROS stack on your computer in simulation mode.

Package Documentation
*********************

.. rosdoc:: /niryo_robot_rpi 
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_rpi


