Niryo_robot_sound
========================================

This package deals with the sound of the robot.


Sound Node
--------------------------
The ROS Node is made of services to play, stop, import and delete a sound on the robot. It is also possible to set the volume of the robot. 

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters - Sound
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here is a list of the different parameters that allow you to adjust the default settings of the robot and the system sounds.

.. list-table:: Parameters of the volume Sound component
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Description
      - Default value

   *  - ``default_volume``
      - Default volume on the real robot
      - 100
   *  - ``default_volume_simulation``
      - Default volume in simulation
      - 10
   *  - ``min_volume``
      - Minimum volume of the robot
      - 0
   *  - ``max_volume``
      - Maximum volume of the robot
      - 200
   *  - ``volume_file_path``
      - File where the volume of the real robot set by the user is stored
      - "~/niryo_robot_saved_files/robot_sound_volume.txt"
   *  - ``volume_file_path_simulation``
      - File where the volume in simulation set by the user is stored
      - "~/.niryo/simulation/robot_sound_volume.txt"

.. list-table:: Parameters of the Sound component
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Name
      - Description
      - Default value

   *  - ``path_user_sound``
      - Default volume on the real robot
      - "~/niryo_robot_saved_files/niryo_robot_user_sounds"
   *  - ``path_user_sound_simulation``
      - Default volume in simulation
      - "~/.niryo/simulation/niryo_robot_user_sounds"
   *  - ``path_robot_sound``
      - Minimum volume of the robot
      - "niryo_robot_state_sounds"
   *  - ``robot_sounds/error_sound``
      - Sound played when an error occurs
      - error.wav
   *  - ``robot_sounds/turn_on_sound``
      - Sound played at the start-up of the robot
      - booting.wav
   *  - ``robot_sounds/turn_off_sound``
      - Sound played at shutdown
      - stop.wav
   *  - ``robot_sounds/connection_sound``
      - Sound played an Niryo Studio connection
      - connected.wav
   *  - ``robot_sounds/robot_ready_sound``
      - Sound played when the robot is ready
      - ready.wav
   *  - ``robot_sounds/calibration_sound``
      - Sound played at start of calibration
      - calibration.wav


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
              <source src="../../../_static/audio/robot_error.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>
   *  - Shutdown
      - Sound played at shutdown
      - .. raw:: html

            <audio controls="controls">
              <source src="../../../_static/audio/stop.wav" type="audio/wav">
              Your browser does not support the <code>audio</code> element.
            </audio>


Publisher - Sound
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Sound Package's Publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot_sound/sound``
      -  :std_msgs:`std_msgs/String<String>`
      -  Publisesh the sound being played
   *  -  ``/niryo_robot_sound/volume``
      -  :std_msgs:`std_msgs/UInt8<UInt8>`
      -  Publishes the volume of the robot
   *  -  ``/niryo_robot_sound/sound_database``
      -  :ref:`SoundList<source/stack/high_level/niryo_robot_sound:SoundList (Message)>`
      -  Publishes the sounds (and their duration) on the robot


Services - Sound
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Sound Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot_sound/play``
      -  :ref:`PlaySound<source/stack/high_level/niryo_robot_sound:PlaySound (Service)>`
      -  Plays a sound from the robot database
   *  -  ``/niryo_robot_sound/stop``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Stops the sound being played
   *  -  ``/niryo_robot_sound/set_volume``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:SetInt`
      -  Sets the volume percentage between 0 and 200%
   *  -  ``/niryo_robot_sound/text_to_speech``
      -  :ref:`TextToSpeech<source/stack/high_level/niryo_robot_sound:TextToSpeech (Service)>`
      -  Pronouncses a sentence via GTTS
   *  -  ``/niryo_robot_sound/manage``
      -  :ref:`ManageSound<source/stack/high_level/niryo_robot_sound:ManageSound (Service)>`
      -  Stops a sound being played


Subscribers - Sound
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Sound Package subscribers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  - Topic name
      - Message type
      - Description
   *  - ``/niryo_robot_status/robot_status``
      - :ref:`RobotStatus<source/stack/high_level/niryo_robot_status:RobotStatus>`
      - Retrieves the current robot status, and controls the sound accordingly (see :ref:`Niryo_robot_status <source/stack/high_level/niryo_robot_status:Niryo_robot_status>` section)
   *  - ``/niryo_studio_connection``
      - :std_msgs:`std_msgs/Empty<Empty>`
      - Catches Niryo Studio's connection to make a sound.


Dependencies - Sound
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :ref:`niryo_robot_msgs <source/stack/high_level/niryo_robot_msgs:Niryo_robot_msgs>`
- :ref:`niryo_robot_status <source/stack/high_level/niryo_robot_status:Niryo_robot_status>`

Services & Messages files - Sound
----------------------------------------------

SoundList (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_sound/msg/SoundList.msg
   :language: rostype

SoundObject (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_sound/msg/SoundObject.msg
   :language: rostype


ManageSound (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_sound/srv/ManageSound.srv
   :language: rostype

PlaySound (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_sound/srv/PlaySound.srv
   :language: rostype

TextToSpeech (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_sound/srv/TextToSpeech.srv
   :language: rostype

Sound API functions
------------------------------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script playing sound via Python ROS Wrapper will look like: ::

    from niryo_robot_led_ring.api import SoundRosWrapper

    sound = SoundRosWrapper()
    sound.play(sound.sounds[0])

| This class allows you to control the sound of the robot via the internal API.

List of functions subsections:

.. contents::
   :local:
   :depth: 1

.. automodule:: niryo_robot_sound.api.sound_ros_wrapper
   :members: SoundRosWrapper

Play sound
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: SoundRosWrapper
    :members: play, stop, set_volume, say
    :member-order: bysource

Sound database
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: SoundRosWrapper
    :members: sounds, get_sound_duration, delete_sound, import_sound
    :member-order: bysource

.. |namespace| replace:: /niryo_robot_sound/
.. |namespace_emphasize| replace:: ``/niryo_robot_sound/``
.. |package_path| replace:: ../../../../niryo_robot_sound
