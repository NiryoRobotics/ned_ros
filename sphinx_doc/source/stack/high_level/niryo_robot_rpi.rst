Niryo_robot_rpi
========================================

This package deals with Raspberry Pi related stuff (Button, fans, I/O, leds, ...).


Raspberry Pi Node
--------------------------
The ROS Node manages the following components:

- Physical top button: executes actions when the button is pressed.
- Digital I/O panel: gets commands and sends the current state of digital I/Os. Also controls tools like the Electromagnet.
- Analog I/O panel: gets commands and sends the current state of analog I/Os.
- End Effector I/O panel: gets commands and sends the current state of the digital I/Os of the end effector panel on Ned2. Also controls tools like the Electromagnet.
- Robot fans.
- Led: sets the LED color.
- Shutdown Manager: shutdown or reboot the Raspberry.
- ROS log: can remove all previous logs on start_up to prevent a lack of disk space in the long run (SD cards do not have infinite storage).

It belongs to the ROS namespace: |namespace_emphasize|.

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
      -  :ref:`PausePlanExecution<source/stack/high_level/niryo_robot_arm_commander:PausePlanExecution>`
      -  Publishes the current execution state launched when button is pressed
   *  -  ``/niryo_robot/blockly/save_current_point``
      -  :std_msgs:`std_msgs/Int32<Int32>`
      -  Publishes current point when user is in Blockly page to save block by pressing button
   *  -  ``/niryo_robot/rpi/is_button_pressed``
      -  :std_msgs:`std_msgs/Bool<Bool>`
      -  Publishes the button state (true if pressed)
   *  -  ``digital_io_state``
      -  :ref:`DigitalIOState<source/stack/high_level/niryo_robot_rpi:DigitalIOState (Topic)>`
      -  Publishes the digital I/Os state by giving for each it's pin / name / mode / state
   *  -  ``analog_io_state``
      -  :ref:`AnalogIOState<source/stack/high_level/niryo_robot_rpi:AnalogIOState (Topic)>`
      -  Publishes the analog I/Os state by giving for each it's pin / name / mode / state
   *  -  ``/niryo_robot/rpi/led_state``
      -  :std_msgs:`std_msgs/Int8<Int8>`
      -  Publishes the current LED color
   *  -  ``ros_log_status``
      -  :ref:`LogStatus<source/stack/high_level/niryo_robot_rpi:LogStatus (Topic)>`
      -  Publishes the current log status (log size / available disk / boolean if should delete ros log on startup)

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
      -  :ref:`SetInt<source/stack/high_level/niryo_robot_msgs:SetInt>`
      -  Shutdowns the Raspberry Pi
   *  -  ``/niryo_robot/rpi/change_button_mode``
      -  :ref:`SetInt<source/stack/high_level/niryo_robot_msgs:SetInt>`
      -  Changes top button mode (autorun program, blockly, nothing, ...)
   *  -  ``get_analog_io``
      -  :ref:`GetAnalogIO<source/stack/high_level/niryo_robot_rpi:GetAnalogIO (Service)>`
      -  Gets analog IO state list
   *  -  ``get_digital_io``
      -  :ref:`GetDigitalIO<source/stack/high_level/niryo_robot_rpi:GetDigitalIO (Service)>`
      -  Gets digital IO state list
   *  -  ``set_analog_io``
      -  :ref:`SetAnalogIO<source/stack/high_level/niryo_robot_rpi:SetAnalogIO (Service)>`
      -  Sets an analog IO to the given value
   *  -  ``set_digital_io``
      -  :ref:`SetDigitalIO<source/stack/high_level/niryo_robot_rpi:SetDigitalIO (Service)>`
      -  Sets a digital IO to the given value
   *  -  ``set_digital_io_mode``
      -  :ref:`SetDigitalIO<source/stack/high_level/niryo_robot_rpi:SetIOMode (Service)>`
      -  Sets a digital IO to the given mode
   *  -  ``set_led_state``
      -  :ref:`std_msgs/SetInt<source/stack/high_level/niryo_robot_msgs:SetInt>`
      -  Sets LED state
   *  -  ``set_led_custom_blinker``
      -  :ref:`LedBlinker<source/stack/high_level/niryo_robot_rpi:LedBlinker (Service)>`
      -  Sets the LED in blink mode with the color given
   *  -  ``purge_ros_logs``
      -  :ref:`SetInt<source/stack/high_level/niryo_robot_msgs:SetInt>`
      -  Purges ROS log
   *  -  ``set_purge_ros_log_on_startup``
      -  :ref:`SetInt<source/stack/high_level/niryo_robot_msgs:SetInt>`
      -  Modifies the permanent settings that tell if the robot should purge its ROS log at each boot

Dependencies - Raspberry Pi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :msgs_index:`actionlib_msgs`
- :msgs_index:`sensor_msgs`
- :doc:`niryo_robot_msgs`
- :doc:`niryo_robot_arm_commander`


- `Adafruit-GPIO==1.0.3 <https://github.com/adafruit/Adafruit_Python_GPIO>`_
- `Adafruit-PureIO==1.0.1 <https://github.com/adafruit/Adafruit_Python_PureIO/tree/1.0.1>`_
- `Adafruit-BBIO==1.0.9 <https://github.com/adafruit/adafruit-beaglebone-io-python/tree/1.0.9>`_
- `Adafruit-ADS1x15==1.0.2 <https://github.com/adafruit/Adafruit_Python_ADS1x15>`_
- `board==1.0 <https://github.com/tjguk/dojo-board>`_
- `smbus==1.1.post2 <https://i2c.wiki.kernel.org/index.php/I2C_Tools>`_
- `smbus2==0.4.1 <https://github.com/kplindegaard/smbus2/tree/0.4.1>`_
- `spidev==3.5 <https://github.com/doceme/py-spidev>`_


Services files - Raspberry Pi
----------------------------------------------

ChangeMotorConfig (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/ChangeMotorConfig.srv
   :language: rostype

GetAnalogIO (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/GetAnalogIO.srv
   :language: rostype

GetDigitalIO (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/GetDigitalIO.srv
   :language: rostype

LedBlinker (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/LedBlinker.srv
   :language: rostype

SetDigitalIO (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/SetDigitalIO.srv
   :language: rostype

SetAnalogIO (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/SetAnalogIO.srv
   :language: rostype

SetIOMode (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/SetIOMode.srv
   :language: rostype

SetPullup (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/srv/SetPullup.srv
   :language: rostype

Messages files - Raspberry Pi
----------------------------------------------


AnalogIO
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/msg/AnalogIO.msg
   :language: rostype

AnalogIOState (Topic)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/msg/AnalogIOState.msg
   :language: rostype

DigitalIO
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/msg/DigitalIO.msg
   :language: rostype

DigitalIOState (Topic)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/msg/DigitalIOState.msg
   :language: rostype


LogStatus (Topic)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_rpi/msg/LogStatus.msg
   :language: rostype

.. |namespace| replace:: /niryo_robot_rpi/
.. |namespace_emphasize| replace:: ``/niryo_robot_rpi/``