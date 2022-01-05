Niryo robot LED Ring package
========================================

| This package is the one managing the LED Ring of Ned2.

| It is composed of one node, receiving commands and the current robot status, and publishing LED Ring states.
| The LED Ring is composed of 30 WS2811 RGB LESs, controlled by the package with the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_.

.. todo:: translate this documentation in French

LED Ring node
--------------------------
The ROS Node is made to manage the LED Ring state, and to publish its currents status and state
on ROS topics.
It uses a class implementing several animation (11 for now), allowing to control the LED Ring or to
display the current robot status. The LED Ring is also implemented in Rviz.

The LED Ring can either be:

- in **ROBOT STATUS** mode : the LED is displaying the status of the robot.
- | in **USER** mode : the user can control the LED Ring with the several methods implemented, through
   Blockly, Pyniryo or Python ROS Wrapper.


Robot status mode
^^^^^^^^^^^^^^^^^^^

When displaying the **robot status**, the LED Ring has several states which represent different modes and error status. 
Refer to the following table. The node subscribes to the ROS topic ``/niryo_robot_status/robot_status``, published by
the package niryo_robot_status.

.. todo:: add a link to the niryo robot status package once it is documented

.. list-table:: 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Animation and color
      - Description
      - Troubleshooting
   *  - White to blue "Breath"
      - Starting the robot
      - N/A
   *  - Slow blue "chase"
      - Calibration required
      - N/A
   *  - 3 yellow blinks
      - Calibration warning
      - N/A
   *  - Blue "snake"
      - Calibration in progress 
      - N/A
   *  - Solid orange
      - One or more robot axes are outside the motion limits
      - Return the axes to within the limits and start a calibration. 
   *  - 1 white "Breath" lasting 2 seconds then 6 blinks in white
      - Shutdown in progress
      - N/A
   *  - Blinking red
      - Motor error 
      - Check the motor error codes in the "Hardware Status" panel of Niryo Studio.
   *  - Blinking red
      - Very high Raspberry Pi temperature 
      - Check the temperature of the Raspberry on Niryo Studio. |newline| Contact the after-sales service if it persists, as there may be defective fan.
   *  - Solid white
      - Update in progress
      - Nothing 
   *  - N/A
      - Update complete
      - N/A
   *  - "Breath" blue
      - FreeMotion activated
      - N/A
   *  - "Breath" green
      - FreeMotion disabled 
      - N/A
   *  - Solid green 
      - Program running
      - N/A
   *  - "Chase" green 
      - Program paused
      - A short press on the button to resume, a long press |newline| on the top button to cancel the current program.
   *  - "Breath" orange
      - Error in the program 
      - Check the logs on Niryo Studio.
   *  - Solid red
      - Fatal error 
      - Check the logs on Niryo Studio and restart the robot.
   *  - Blinking orange
      - A collision is detected  
      - Perform a new action to remove the warning and check if the |newline| robot has collided with an obstacle.
   *  - Flashing purple 
      - Login to Niryo Studio
      - N/A
   *  - 1 Blinking white
      - Saving a position
      - N/A


User mode 
^^^^^^^^^^^^^^^^^^^

Several animations are implemented to allow the user different ways to control the LED Ring. Refer to the following
table. The node receives commands through the service ``/niryo_robot_led_ring/set_user_animation`` (see :ref:`Services - LED Ring` section)

.. todo:: put this table with gifs also in the Ned 2 user documentation

.. important:: Ned must be in autonomous mode in order to allow the user to control the LED Ring.

.. list-table:: 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Method name
      - Parameters
      - Appearance
      - Gif

   *  - none
      - N/A (No parameters)
      - LEDs are turned off
      -
   *  - solid
      - color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
      - Set the whole LED Ring to the same color at once
      -
   *  - flashing
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | freq (in Hertz, int)
        | iterations (int)
      - | Flashes a color according to a frequency ("freq"). if 
        | iterations is null, flashes endlessly
      -
   *  - alternate
      - | colors_list (list of :std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | The different colors are alternated one after the other, each displayed during
        | "wait_ms" millisecondes. If iterations is 0, alternates indefinitely.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/alternate.gif
   *  - chase
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Movie theater light style chase animation. Each step lasts for "wait_ms" millisecondes.
        | If iterations is 0, do it indefinitely.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/chase.gif
   *  - color_wipe
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
      - | Wipe a color across the LED Ring, lighting a LED at each step (of duration "wait_ms").
        | Similar to go_up, but LEDs are not turned off at the end.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/color_wipe.gif
   *  - rainbow
      - | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Draws rainbow that fades across all LEDs at once.
        | If iterations is 0, do it indefinitely
      -  .. figure:: ../../../images/stack/high_level/gif_led_ring/rainbow.gif
   *  - rainbow_cycle
      - | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Draw rainbow that uniformly distributes itself across all LEDs
        | If iterations is 0, do it indefinitely.
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/rainbow_cycle.gif
   *  - rainbow_chase
      - | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Rainbow chase animation.
        | If iterations is 0, do it indefinitely. One iteration corresponds
        | to one full rainbow fading accross LEDs, unlike the chase method
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/rainbow_chase.gif
   *  - go_up
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | LEDs turn on like a loading circle until lighting up the whole LED Ring.
        | and are then all turned off at the same time. If iterations is 0, do it indefinitely
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/goup.gif
   *  - go_up_and_down
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Like go_up, but LEDs are turned off the same way they are turned on.
        | If iterations is 0, do it indefinitely
      - .. figure:: ../../../images/stack/high_level/gif_led_ring/goupanddown.gif

.. note:: When displaying the robot status, the LED Ring commander uses those methods, with the defaults parameters defined below.

The namespace used is: |namespace_emphasize|

Parameters - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Firstly, the LED Ring component, controlled with  the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_, through
the Python class PixelStrip, is parameterizable. Default parameters are set in the `led_strim_params.yaml` file of the  `/config` folder of the package

.. todo:: the led_pin might change when adding the led ring directly on Ned

.. list-table:: Parameters of the LED Ring component
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
      - 18
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
      - False
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

   *  - ``default_flashing_frequency``
      - Flash frequency for the flashing animation, in Hertz
      - 4
   *  - ``default_alternate_wait_ms``
      - | Time between each color alternation, for the alternate animation,
        | in millisecondes
      - 500
   *  - ``default_chase_wait_ms``
      - Time between each step of the chase animation, in millisecondes
      - 50
   *  - ``default_colorwipe_wait_ms``
      - Time between each step of the color wipe animation, in millisecondes
      - 50
   *  - ``default_rainbow_wait_ms``
      - Time between each step of the rainbow animation, in millisecondes
      - 20
   *  - ``default_rainbowcycle_wait_ms``
      - Time between each step of the rainbow cycle animation, in millisecondes
      - 20
   *  - ``default_rainbowchase_wait_ms``
      - Time between each step of the rainbow chase animation, in millisecondes
      - 50
   *  - ``default_goup_wait_ms``
      - Time between each step of the go up animation, in millisecondes
      - 50
   *  - ``default_goupanddown_wait_ms``
      - Time between each step of the go up and down animation, in millisecondes
      - 50


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
      - :ref:`LEDUser<LEDUser (Service)>`
      - | Allows the user to control the LED Ring, with implemented animations. A new request
        | will **interrupt** the previous one, if still playing. Depending on the ``wait`` boolean field
        | and the ``iterations`` field of the request, the service will either **answer immediately** after
        | launching the animation, or **wait for the animation to finish** to answer.


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
      - :ref:`LEDRingStatus<LEDRingStatus (Message)>`
      - | Publishes the **status** of the LED Ring, providing information on the **current mode** 
        | (displaying robot status or controlled by user if the robot works in AUTONOMOUS mode),
        | the **current animation played** and the animation color (except for rainbow methods, where
        | the animation color is not defined). Publishes every time at least **one field changed**.
   *  - ``led_ring_current_state``
      - :ref:`LEDRingCurrentState<LEDRingCurrentState (Message)>`
      - | Publishes the current **state** of each LED, as a list of colors of size 30. Publishes everytime
        | the state of at least one LED changes.
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
      - niryo_robot_status/RobotStatus
      - Retrieves the current robot status, and control LED accordingly (see :ref:`Robot status mode` section)

.. todo:: add a link to the robot status message once it is documented.


Dependencies - LED Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :ref:`niryo_robot_msgs <source/stack/high_level/niryo_robot_msgs:Niryo_robot_msgs>`
- :msgs_index:`std_msgs`
- :msgs_index:`visualization_msgs`


Action, services & messages files - LED Ring
------------------------------------------------------

LEDUser (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/srv/LEDUser.srv
   :language: rostype

LEDRingStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/msg/LEDRingStatus.msg
   :language: rostype

LEDRingCurrentState (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/msg/LEDRingCurrentState.msg
   :language: rostype

LEDRingAnimation (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_led_ring/msg/LEDRingAnimation.msg
   :language: rostype






.. |namespace| replace:: /niryo_robot_led_ring/
.. |namespace_emphasize| replace:: ``/niryo_robot_led_ring/``
.. |package_path| replace:: ../../../../niryo_robot_led_ring
.. |newline| raw:: html

    <br>