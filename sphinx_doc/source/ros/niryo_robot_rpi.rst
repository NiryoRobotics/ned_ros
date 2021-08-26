Niryo_robot_rpi
========================================

This package deals with Raspberry Pi related stuff (Button, fans, I/O, leds, ...).


Raspberry Pi Node
--------------------------
The ROS Node manages the following components:

- Physical top button: executes actions when the button is pressed.
- Digital I/O panel: gets commands and sends the current state of digital I/Os. Also controls tools like the Electromagnet.
- Robot fans.
- Led: sets the LED color.
- ROS log: can remove all previous logs on startup to prevent a lack of disk space in the long run (SD cards do not have infinite storage).

The namespace used is: |namespace_emphasize|

**Note that this package should not be used when you are using Ned ROS stack on your computer in simulation mode. Executes actions when the button is pressed.**

Publisher - Raspberry Pi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: RPI Package's Publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``pause_state``
      -  :ref:`PausePlanExecution<source/ros/niryo_robot_arm_commander:PausePlanExecution (Message)>`
      -  Publish the current execution state launched when button is pressed
   *  -  ``/niryo_robot/blockly/save_current_point``
      -  :std_msgs:`std_msgs/Int32<Int32>`
      -  Publish current point when user is in Blockly page to save block by pressing button
   *  -  ``/niryo_robot/rpi/is_button_pressed``
      -  :std_msgs:`std_msgs/Bool<Bool>`
      -  Publish the button state (true if pressed)
   *  -  ``digital_io_state``
      -  :ref:`DigitalIOState<source/ros/niryo_robot_rpi:DigitalIOState (Service)>`
      -  Publish the I/Os state by giving for each it's pin / name / mode / state
   *  -  ``/niryo_robot/rpi/led_state``
      -  :std_msgs:`std_msgs/Int8<Int8>`
      -  Publish the current led color
   *  -  ``ros_log_status``
      -  :ref:`LogStatus<source/ros/niryo_robot_rpi:LogStatus (Service)>`
      -  Publish the current log status (log size / available disk / boolean if should delete ros log on startup)

Services - Raspberry Pi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: RPI Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``shutdown_rpi``
      -  :ref:`SetInt<source/ros/niryo_robot_msgs:SetInt>`
      -  Shutdown the Raspberry Pi
   *  -  ``/niryo_robot/rpi/change_button_mode``
      -  :ref:`SetInt<source/ros/niryo_robot_msgs:SetInt>`
      -  Change top button mode (autorun program, blockly, nothing, ...)
   *  -  ``get_digital_io``
      -  :ref:`GetDigitalIO<source/ros/niryo_robot_rpi:GetDigitalIO (Service)>`
      -  Get digital IO state list
   *  -  ``set_digital_io_mode``
      -  :ref:`SetDigitalIO<source/ros/niryo_robot_rpi:SetDigitalIO (Service)>`
      -  Set a digital IO to the mode given
   *  -  ``set_digital_io_state``
      -  :ref:`SetDigitalIO<source/ros/niryo_robot_rpi:SetDigitalIO (Service)>`
      -  Set a digital IO to the state given
   *  -  ``set_led_state``
      -  :ref:`std_msgs/SetInt<source/ros/niryo_robot_msgs:SetInt>`
      -  Set led state
   *  -  ``set_led_custom_blinker``
      -  :ref:`LedBlinker<source/ros/niryo_robot_rpi:LedBlinker (Service)>`
      -  Set the led in blink mode with the color given
   *  -  ``purge_ros_logs``
      -  :ref:`SetInt<source/ros/niryo_robot_msgs:SetInt>`
      -  Purge ROS log
   *  -  ``set_purge_ros_log_on_startup``
      -  :ref:`SetInt<source/ros/niryo_robot_msgs:SetInt>`
      -  Modify the permanent settings that tells if robot should purge it's ROS log at each boot

Dependencies - Raspberry Pi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :msgs_index:`actionlib_msgs`
- :msgs_index:`sensor_msgs`
- :doc:`niryo_robot_msgs`
- :doc:`niryo_robot_arm_commander`

Services & Messages files - Raspberry Pi
----------------------------------------------

ChangeMotorConfig (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_rpi/srv/ChangeMotorConfig.srv
   :language: rostype


GetDigitalIO (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_rpi/srv/GetDigitalIO.srv
   :language: rostype

LedBlinker (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_rpi/srv/LedBlinker.srv
   :language: rostype

SetDigitalIO (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_rpi/srv/SetDigitalIO.srv
   :language: rostype

DigitalIOState (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_rpi/msg/DigitalIOState.msg
   :language: rostype

LogStatus (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_rpi/msg/LogStatus.msg
   :language: rostype

.. |namespace| replace:: /niryo_robot_rpi/
.. |namespace_emphasize| replace:: ``/niryo_robot_rpi/``