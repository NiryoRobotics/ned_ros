Niryo robot sound
#################

This package contains services to play, stop, import and delete a sound on the robot. It is also possible to set the volume of the robot. 

It belongs to the ROS namespace: |namespace_emphasize|.

.. |namespace_emphasize| replace:: ``/niryo_robot_sound``

Sounds
******

.. list-table:: State sounds
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - State
      - Description
      - Sound

   *  - Booting
      - Sound played while booting
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/booting.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Ready
      - Sound played when the robot is ready after booting
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/ready.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Calibration
      - Sound played at start of calibration
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/calibration.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Connected
      - Notify of a connection to Niryo Studio
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/connected.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Reboot
      - Sound played at start of a motor reboot
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/reboot.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Warn
      - Sound played when a warning occurs
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/warn.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Error
      - Sound played when a robot/motor/raspberry/program/overheating error occurs
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/error.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Shutdown
      - Sound played at shutdown
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/stop.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>

Sound API functions
*******************

Sound ROS wrapper
-----------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script playing sound via Python ROS Wrapper will looks like: 

.. code:: python

    from niryo_robot_sound.api import SoundRosWrapper

    sound = SoundRosWrapper()
    sound.play(sound.sounds[0])

| This class allows you to control the sound of the robot via the internal API.

API list
--------

.. automodule:: niryo_robot_sound.api.sound_ros_wrapper
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Package Documentation
*********************

.. rosdoc:: /niryo_robot_sound 
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_sound

