Niryo_robot_led_ring
========================================

| This package is the one managing the LED Ring of Ned2.

| It is composed of one node, receiving commands and the current robot status, and publishing LED Ring states.

| The LED Ring is composed of 30 WS2811 RGB LEDs, controlled by the package with the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_.


LED Ring node
--------------------------
The ROS Node is made to manage the LED Ring state, and to publish its currents status and state
on ROS topics.
It uses a class implementing several animation (11 for now), allowing to control the LED Ring or to
display the current robot status. The LED Ring is also implemented in Rviz.

The LED Ring can either be:

- in **ROBOT STATUS** mode: the LED is displaying the status of the robot.
- in **USER** mode: the user can control the LED Ring with the several methods implemented, through
    `Blockly <https://docs.niryo.com/product/niryo-studio/v3.2.1/en/source/blockly_api.html>`_ ,
    `Pyniryo <https://docs.niryo.com/dev/pyniryo/v1.0.5/en/index.html>`_ or 
    :ref:`Python ROS Wrapper <source/python_ros_wrapper/ros_wrapper_doc:Python ROS Wrapper documentation>` .


Robot status mode
^^^^^^^^^^^^^^^^^^^

When displaying the **robot status**, the LED Ring has several states which represent different modes and error status. 
Refer to the following table. The node subscribes to the ROS topic ``/niryo_robot_status/robot_status``, published by
the package :ref:`RobotStatus<source/stack/high_level/niryo_robot_status:RobotStatus>`.

.. list-table::
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Animation and color
      - Description
      - Troubleshooting
   *  - White :ref:`Breath <Breath>`
      - Robot is booting
      - N/A
   *  - Blue :ref:`Chase <Chase>`
      - Calibration is needed
      - Press the *Custom* button, or launch a calibration
   *  - Blue :ref:`Snake <Snake>`
      - Calibration in progress
      - N/A
   *  - Blue :ref:`Breath <Breath>`
      - Free Motion enabled
      - N/A
   *  - 3 Yellow :ref:`Flashing <Flashing>`
      - Calibration start
      - N/A
   *  - Green :ref:`Breath <Breath>`
      - Free Motion disabled, torque enabled
      - N/A
   *  - :ref:`Solid <Solid>` Green
      - Program in progress
      - N/A
   *  - Green :ref:`Chase <Chase>`
      - Program paused
      - Long press on the TOP button to cancel the program, short press to resume
   *  - Orange :ref:`Breath <Breath>`
      - Program execution error
      - Launch a new action to clear this state
   *  - :ref:`Flashing <Flashing>` Orange
      - Collision
      - Launch a new action to clear this state
   *  - :ref:`Solid <Solid>` Orange
      - Joint out of bounds
      - Switch to Free Motion mode to bring the joints within limits.
   *  - 1 Purple :ref:`Flashing <Flashing>`
      - New connection form Niryo Studio
      - N/A
   *  - 2 Purple :ref:`Flashing <Flashing>`
      - Save a robot positions from the 'Save' button
      - N/A
   *  - :ref:`Flashing <Flashing>` Red
      - Motor error / Raspberry overheating
      - Please check the error on Niryo Studio.
   *  - :ref:`Solid <Solid>` Red
      - ROS Crash
      - Please restart the robot.

User mode 
^^^^^^^^^^^^^^^^^^^

Several animations are implemented to allow the user different ways to control the LED Ring. Refer to the following
table. The node receives commands through the service ``/niryo_robot_led_ring/set_user_animation`` (see :ref:`the service section<source/stack/high_level/niryo_robot_led_ring:Services - Led Ring>`)

.. important:: Ned must be in autonomous mode in order to allow the user to control the LED Ring.

.. list-table:: 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Animation
      - Appearance
      - Gif

   *  - _`None`
      - LEDs are turned off
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_none.gif
           :height: 250px
   *  - _`Solid`
      - Set the whole LED Ring to the same color at once
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_solid.gif
           :height: 250px
   *  - _`Flashing`
      - | Flashes a color according to a frequency
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_flash.gif
           :height: 250px
   *  - _`Alternate`
      - | The different colors are alternated one after the other.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_alternate.gif
           :height: 250px
   *  - _`Chase`
      - | Movie theater light style chase animation.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_chase.gif
           :height: 250px
   *  - _`Color Wipe`
      - | Wipe a color across the LED Ring.
        | Similar to go_up, but LEDs are not turned off at the end.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_wipe.gif
           :height: 250px
   *  - _`Rainbow`
      - | Draws rainbow that fades across all LEDs at once.
      -  .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_rainbow.gif
           :height: 250px
   *  - _`Rainbow cycle`
      - | Draw rainbow that uniformly distributes itself across all LEDs.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_rainbow_cycle.gif
           :height: 250px
   *  - _`Rainbow chase`
      - | Rainbow chase animation.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_rainbow_chase.gif
           :height: 250px
   *  - _`Go up`
      - | LEDs turn on like a loading circle until lighting up the whole LED Ring.
        | and are then all turned off at the same time.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_goup.gif
           :height: 250px
   *  - _`Go up and down`
      - | Like go_up, but LEDs are turned off the same way they are turned on.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_goupdown.gif
           :height: 250px
   *  - _`Breath`
      - | Variation of light intensity to imitate breathing.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_breath.gif
           :height: 250px
   *  - _`Snake`
      - | Luminous snake that turns around the LED Ring.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/ned_led_ring_snake.gif
           :height: 250px


.. note:: When displaying the robot status, the LED Ring commander uses those methods, with the default parameters defined below.

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Firstly, the LED Ring component, controlled with  the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_, through
the Python class PixelStrip, is parameterizable. Default parameters are set in the `led_strim_params.yaml` file of the  `/config` folder of the package

.. list-table:: Parameters of the Led Ring component
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Description
      - Default value

   *  - ``led_count``
      - Number of LED pixels in the LED Ring
      - 30
   *  - ``led_pin``
      - | Raspberry Pi GPIO pin connected to the pixels 
        | It must support PWM.
      - 13
   *  - ``led_freq_hs``
      - LED signal frequency in Hertz
      - 800khz
   *  - ``led_dma``
      - DMA channel to use for generating signal
      - 10
   *  - ``led_brightness``
      - LEDs brightness. Set to 0 for darkest and 255 for brightest
      - 255
   *  - ``led_invert``
      - True to invert the signal (when using NPN transistor level shift)
      - True
   *  - ``led_channel``
      - the PWM channel to use
      - 0

Another configuration file, the `led_ring_params.yaml`, sets the default parameters of LED Ring animations.

.. list-table:: Parameters of the LED Ring animations
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Description
      - Default value

   *  - ``default_flashing_period``
      - Default :ref:`Flashing <Flashing>`  animation period in seconds
      - 0.25
   *  - ``default_alternate_period``
      - Default :ref:`Alternate <Alternate>`  animation period in seconds
      - 1
   *  - ``default_chase_period``
      - Default :ref:`Chase <Chase>`  animation period in seconds
      - 4
   *  - ``default_colorwipe_period``
      - Default :ref:`Wipe <Color Wipe>`  animation period in seconds
      - 5
   *  - ``default_rainbow_period``
      - Default :ref:`Rainbow <Rainbow>`  animation period in seconds
      - 5
   *  - ``default_rainbowcycle_period``
      - Default :ref:`Rainbow cycle <Rainbow cycle>`  animation period in seconds
      - 5
   *  - ``default_rainbowchase_period``
      - Default :ref:`Rainbow chase <Rainbow chase>`  animation period in seconds
      - 5
   *  - ``default_goup_period``
      - Default :ref:`Go up <Go up>`  animation period in seconds
      - 5
   *  - ``default_goupanddown_period``
      - Default :ref:`Go up and down <Go up and down>`  animation period in seconds
      - 5
   *  - ``default_breath_period``
      - Default :ref:`Breath <Breath>`  animation period in seconds
      - 4
   *  - ``default_snake_period``
      - Default :ref:`Snake <Snake>` animation period in seconds
      - 1.5
   *  - ``led_offset``
      - Offset ID between the LED with the ID 0 and the ID of the LED at the back of the robot.
      - 8
   *  - ``simulation_led_ring_markers_publish_rate``
      - Rviz LED ring markers publishinf rate in simulation mode
      - 20
   *  - ``led_ring_markers_publish_rate``
      - Rviz LED ring markers publishing rate on a real robot
      - 5


Services - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^


The ROS node implements one service, designed for the user to control the LED Ring.

.. list-table:: LED Ring Package services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Message type
      - Description
   *  - ``set_user_animation``
      - :ref:`LedUser<source/stack/high_level/niryo_robot_led_ring:LedUser (Service)>`
      - | Allows user to control the LED Ring, with implemented animations. A new request
        | will **interrupt** the previous one, if still playing. Depending on the ``wait`` boolean field
        | and the ``iterations`` field of the request, the service will either **answer immediately** after
        | launching the animation, or **wait for the animation to finish** to answer.
   *  - ``set_led_color``
      - :ref:`SetLedColor<source/stack/high_level/niryo_robot_led_ring:SetLedColor (Service)>`
      - Lights up a LED identified by an ID


Publishers - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: LED Ring Package publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Message type
      - Description
   *  - ``led_ring_status``
      - :ref:`LedRingStatus<source/stack/high_level/niryo_robot_led_ring:LedRingStatus (Message)>`
      - | Publishes the **status** of the LED Ring, providing information on the **current mode** 
        | (displaying robot status or controlled by user if the robot works in AUTONOMOUS mode),
        | the **current animation played** and the animation color (except for rainbow methods, where
        | the animation color is not defined). Publishes every time at least **one field changed**.
   *  - ``visualization_marker_array``
      - :visualization_msgs:`visualization_msgs/MarkerArray<MarkerArray>`
      - | Publishes shapes representing LEDs when Ned is used in simulation with **Rviz**, 
        | as a list of 30 :visualization_msgs:`visualization_msgs/Marker<Marker>` of size 30. 


Subscribers - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. list-table:: LED Ring Package subscribers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Topic name
      - Message type
      - Description
   *  - ``/niryo_robot_status/robot_status``
      - :ref:`RobotStatus<source/stack/high_level/niryo_robot_status:RobotStatus>`
      - Retrieves the current robot status, and control LED accordingly (see :ref:`Niryo_robot_status <source/stack/high_level/niryo_robot_status:Niryo_robot_status>` section)
   *  - ``/niryo_robot/blockly/save_current_point``
      - :std_msgs:`std_msgs/Int32<Int32>`
      - Catches the 'Save Point' action to make the LED ring blink.
   *  - ``/niryo_studio_connection``
      - :std_msgs:`std_msgs/Empty<Empty>`
      - Catches the Niryo Studio connection to make the LED ring blink.


Dependencies - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :ref:`niryo_robot_msgs <source/stack/high_level/niryo_robot_msgs:Niryo_robot_msgs>`
- :msgs_index:`std_msgs`
- :msgs_index:`visualization_msgs`

- `rpi_ws281x==4.3.0 <https://github.com/rpi-ws281x/rpi-ws281x-python/tree/v4.3.0>`_


Services files - LED Ring
------------------------------------------------------

LedUser (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/srv/LedUser.srv
   :language: rostype

SetLedColor (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/srv/SetLedColor.srv
   :language: rostype


Messages files - LED Ring
------------------------------------------------------

LedRingAnimation (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/msg/LedRingAnimation.msg
   :language: rostype

LedRingCurrentState (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/msg/LedRingCurrentState.msg
   :language: rostype

LedRingStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/msg/LedRingStatus.msg
   :language: rostype



LED Ring API functions
------------------------------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script turning on the LED Ring via Python ROS Wrapper will look like: ::

    from niryo_robot_led_ring.api import LedRingRosWrapper

    led_ring = LedRingRosWrapper()
    led_ring.solid(color=[255, 255, 255])


| This class allows you to control the robot via internal API. By controlling, we mean using the LED ring

List of functions subsections:

.. contents::
   :local:
   :depth: 1

.. automodule:: niryo_robot_led_ring.api.led_ring_ros_wrapper
   :members: LedRingRosWrapper

Custom animations functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: LedRingRosWrapper
    :members: set_led_color, custom
    :member-order: bysource

Pre-made animations functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: LedRingRosWrapper
    :members: solid, turn_off, flashing, alternate, chase, wipe, rainbow, rainbow_cycle, rainbow_chase,
              go_up, go_up_down, breath, snake
    :member-order: bysource

.. |namespace| replace:: /niryo_robot_led_ring/
.. |namespace_emphasize| replace:: ``/niryo_robot_led_ring/``
.. |package_path| replace:: ../../../../niryo_robot_led_ring
.. |newline| raw:: html

    <br>