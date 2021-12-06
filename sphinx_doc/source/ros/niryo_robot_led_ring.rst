Niryo robot Led Ring package
========================================

| This package is the one managing the Led Ring of Ned.
| It is composed of one node, receiving commands and the current robot status, and publishing Led Ring states.
| The Led Ring is composed of 30 WS2811 RGB Leds, controlled by the package with the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_.

.. todo:: translate this documentation in French

Led Ring node
--------------------------
The ROS Node is made to manage the Led Ring state, and to publish its currents status and state
on ROS topics.
It uses a class implementing several animation (11 for now), allowing to control the Led Ring or to
display the current robot status. The Led Ring is also implemented in Rviz.

The Led Ring can either be:

- in **ROBOT STATUS** mode : the Led is displaying the status of the robot.
- | in **USER** mode : the user can control the Led Ring with the several methods implemented, through
   Blockly, Pyniryo or Python ROS Wrapper.


Robot status mode
^^^^^^^^^^^^^^^^^^^

When displaying the **robot status**, the Led Ring has several states which represent different modes and error status. 
Refer to the following table. The node subscribes to the ROS topic ``/niryo_robot_status/robot_status``, published by
the package niryo_robot_status.

.. todo:: add a link to the niryo robot status package once it is documented

.. todo:: update the table if we add colors for other robots status

.. list-table:: 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Animation and color
      - Description
      - Troubleshooting
   *  - Solid green
      - Standby in manual mode
      - N/A
   *  - Flashing green
      - Robot moving in manual mode
      - N/A
   *  - Alternating green and red
      - Robot moving error
      - Press Learning Mode button to troubleshoot the error 
   *  - White chase
      - Robot booting or updating
      - N/A
   *  - Solid white
      - Learning mode
      - N/A
   *  - Alternating white and yellow
      - Learning mode warning
      - N/A
   *  - Alternating white and red
      - Learning mode error
      - N/A
   *  - Solid blue
      - | Running autonomous mode (robot controlled through Blockly, 
        | PyNiryo or Python ROS Wrapper)
      - N/A
   *  - Alternating blue and red
      - Running autonomous error
      - N/A
   *  - Alternating blue and yellow
      - | Running autonomous warning 
        | or
        | Learning mode in autonomous mode warning
      - N/A
   *  - Alternating blue and white
      - Learning mode in autonomous mode
      - N/A


User mode 
^^^^^^^^^^^^^^^^^^^

Several animations are implemented to allow the user different ways to control the Led Ring. Refer to the following
table. The node receives commands through the service ``/niryo_robot_led_ring/set_user_animation`` (see :ref:`Services - Led Ring` section)

.. todo:: put this table with gifs also in the Ned 2 user documentation

.. important:: Ned must be in autonomous mode in order to allow the user to control the Led Ring.

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
      - Leds are turned off
      -
   *  - solid
      - color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
      - Set the whole Led Ring to the same color at once
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
      - .. figure:: ../../images/gif_led_ring/alternate.gif
   *  - chase
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Movie theater light style chase animation. Each step lasts for "wait_ms" millisecondes.
        | If iterations is 0, do it indefinitely.
      - .. figure:: ../../images/gif_led_ring/chase.gif
   *  - color_wipe
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
      - | Wipe a color across the Led Ring, lighting a Led at each step (of duration "wait_ms").
        | Similar to go_up, but Leds are not turned off at the end.
      - .. figure:: ../../images/gif_led_ring/color_wipe.gif
   *  - rainbow
      - | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Draws rainbow that fades across all Leds at once.
        | If iterations is 0, do it indefinitely
      -  .. figure:: ../../images/gif_led_ring/rainbow.gif
   *  - rainbow_cycle
      - | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Draw rainbow that uniformly distributes itself across all Leds
        | If iterations is 0, do it indefinitely.
      - .. figure:: ../../images/gif_led_ring/rainbow_cycle.gif
   *  - rainbow_chase
      - | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Rainbow chase animation.
        | If iterations is 0, do it indefinitely. One iteration corresponds
        | to one full rainbow fading accross Leds, unlike the chase method
      - .. figure:: ../../images/gif_led_ring/rainbow_chase.gif
   *  - go_up
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Leds turn on like a loading circle until lighting up the whole Led Ring.
        | and are then all turned off at the same time. If iterations is 0, do it indefinitely
      - .. figure:: ../../images/gif_led_ring/goup.gif
   *  - go_up_and_down
      - | color (:std_msgs:`std_msgs/ColorRGBA<ColorRGBA>`)
        | wait_ms (in millisecondes, int)
        | iterations (int)
      - | Like go_up, but Leds are turned off the same way they are turned on.
        | If iterations is 0, do it indefinitely
      - .. figure:: ../../images/gif_led_ring/goupanddown.gif

.. note:: When displaying the robot status, the Led Ring commander uses those methods, with the defaults parameters defined below.

The namespace used is: |namespace_emphasize|

Parameters - Led Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Firstly, the Led Ring component, controlled with  the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_, through
the python class PixelStrip, is parameterizable. Default parameters are set in the `led_strim_params.yaml` file of the  `/config` folder of the package

.. todo:: the led_pin might change when adding the led ring directly on Ned

.. list-table:: Parameters of the Led Ring component
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Description
      - Default value

   *  - ``led_count``
      - Number of LED pixels in the Led Ring
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
      - Leds brightness. Set to 0 for darkest and 255 for brightest
      - 255
   *  - ``led_invert``
      - True to invert the signal (when using NPN transistor level shift)
      - False
   *  - ``led_channel``
      - the PWM channel to use
      - 0

Another configuration file, the `led_ring_params.yaml`, sets the default parameters of Led Ring animations.

.. list-table:: Parameters of the Led Ring animations
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


Services - Led Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ROS node implements one service, designed for the user to control the Led Ring.

.. list-table:: Led Ring Package services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Message type
      - Description
   *  - ``set_user_animation``
      - :ref:`LedUser<LedUser (Service)>`
      - | Allow user to control the Led Ring, with implemented animations. A new request
        | will **interrupt** the previous one, if still playing. Depending on the ``wait`` boolean field
        | and the ``iterations`` field of the request, the service will either **answer immediately** after
        | launching the animation, or **wait for the animation to finish** to answer.


Publishers - Led Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Led Ring Package publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Message type
      - Description
   *  - ``led_ring_status``
      - :ref:`LedRingStatus<LedRingStatus (Message)>`
      - | Publishes the **status** of the Led Ring, providing information on the **current mode** 
        | (displaying robot status or controlled by user if the robot works in AUTONOMOUS mode),
        | the **current animation played** and the animation color (except for rainbow methods, where
        | the animation color is not defined). Publishes every time at least **one field changed**.
   *  - ``led_ring_current_state``
      - :ref:`LedRingCurrentState<LedRingCurrentState (Message)>`
      - | Publishes the current **state** of each Led, as a list of colors of size 30. Publishes everytime
        | the state of at least one Led changes.
   *  - ``visualization_marker_array``
      - :visualization_msgs:`visualization_msgs/MarkerArray<MarkerArray>`
      - | Publishes shapes representing Leds when Ned is used in simulation with **Rviz**, 
        | as a list of 30 :visualization_msgs:`visualization_msgs/Marker<Marker>` of size 30. 


Subscribers - Led Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. list-table:: Led Ring Package subscribers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Topic name
      - Message type
      - Description
   *  - ``/niryo_robot_status/robot_status``
      - niryo_robot_status/RobotStatus
      - Retrieves the current robot status, and control Led accordingly (see :ref:`Robot status mode` section)

.. todo:: add a link to the robot status message once it is documented.


Dependencies - Led Ring
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`
- :msgs_index:`std_msgs`
- :msgs_index:`visualization_msgs`


Action, services & messages files - Led Ring
------------------------------------------------------

LedUser (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_led_ring/srv/LedUser.srv
   :language: rostype

LedRingStatus (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_led_ring/msg/LedRingStatus.msg
   :language: rostype

LedRingCurrentState (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_led_ring/msg/LedRingCurrentState.msg
   :language: rostype

LedRingAnimation (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_led_ring/msg/LedRingAnimation.msg
   :language: rostype






.. |namespace| replace:: /niryo_robot_led_ring/
.. |namespace_emphasize| replace:: ``/niryo_robot_led_ring/``
.. |package_path| replace:: ../../../niryo_robot_led_ring
