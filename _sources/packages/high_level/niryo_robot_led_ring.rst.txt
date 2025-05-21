Niryo robot led ring
####################

This package is in charge of managing Ned robot's led ring.
It is composed of one node, receiving commands and the current robot status, and publishing LED Ring states.

It belongs to the ROS namespace: |namespace_emphasize|.

The LED Ring is composed of 30 WS2811 RGB LEDs, controlled by using the `rpi_ws281x library <https://github.com/rpi-ws281x/rpi-ws281x-python>`_.

The LED Ring can either be controlled in:

#. :ref:`Robot status mode<Robot status mode>`: the LED ring is displaying the status of the robot.
#. :ref:`User mode<User mode>`: the user can control the LED Ring with the several methods implemented:

   #. :blockly:`Blockly <>`
   #. :pyniyro:`Pyniryo <>`
   #. :doc:`Python ROS Wrapper <../ros_wrapper>`

_`Robot status mode`
********************

When displaying the **robot status**, the LED Ring has several states which represent different modes and error status. 
Refer to the following table. The node subscribes to the ROS topic ``/niryo_robot_status/robot_status``, published by
the :doc:`robot status package <niryo_robot_status>`.

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

_`User mode` 
************

Several animations are implemented to allow the user different ways to control the LED Ring. Refer to the following
table. The node receives commands through the service ``/niryo_robot_led_ring/set_user_animation``.

.. important:: The robot must be in autonomous mode in order to allow the user to control the LED Ring.

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
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_none.gif
           :height: 250px
   *  - _`Solid`
      - Set the whole LED Ring to the same color at once
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_solid.gif
           :height: 250px
   *  - _`Flashing`
      - | Flashes a color according to a frequency
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_flash.gif
           :height: 250px
   *  - _`Alternate`
      - | The different colors are alternated one after the other.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_alternate.gif
           :height: 250px
   *  - _`Chase`
      - | Movie theater light style chase animation.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_chase.gif
           :height: 250px
   *  - _`Color Wipe`
      - | Wipe a color across the LED Ring.
        | Similar to go_up, but LEDs are not turned off at the end.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_wipe.gif
           :height: 250px
   *  - _`Rainbow`
      - | Draws rainbow that fades across all LEDs at once.
      -  .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_rainbow.gif
           :height: 250px
   *  - _`Rainbow cycle`
      - | Draw rainbow that uniformly distributes itself across all LEDs.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_rainbow_cycle.gif
           :height: 250px
   *  - _`Rainbow chase`
      - | Rainbow chase animation.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_rainbow_chase.gif
           :height: 250px
   *  - _`Go up`
      - | LEDs turn on like a loading circle until lighting up the whole LED Ring.
        | and are then all turned off at the same time.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_goup.gif
           :height: 250px
   *  - _`Go up and down`
      - | Like go_up, but LEDs are turned off the same way they are turned on.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_goupdown.gif
           :height: 250px
   *  - _`Breath`
      - | Variation of light intensity to imitate breathing.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_breath.gif
           :height: 250px
   *  - _`Snake`
      - | Luminous snake that turns around the LED Ring.
      - .. figure:: /.static/gifs/gif_led_ring/ned_led_ring_snake.gif
           :height: 250px


Led ring API functions
**********************

Led ring ROS wrapper
--------------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script turning on the LED Ring via Python ROS Wrapper will look like:

.. code:: python

    from niryo_robot_led_ring.api import LedRingRosWrapper

    led_ring = LedRingRosWrapper()
    led_ring.solid(color=[255, 255, 255])

| This class allows you to control the led ring via internal API.

API list
--------

.. automodule:: niryo_robot_led_ring.api.led_ring_ros_wrapper
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Package Documentation
*********************

.. rosdoc:: /niryo_robot_led_ring
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_led_ring

.. |namespace_emphasize| replace:: ``/niryo_robot_led_ring``
