Niryo robot Sound package
========================================

This package deals with the sound of the robot.


Sound Node
--------------------------
The ROS Node is made of services to play, stop, import and delete a sound on the robot. It is also possible to set the volume of the robot. 

The namespace used is: |namespace_emphasize|

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
   *  -  ``/niryo_robot_sound/sound_user_state``
      -  :std_msgs:`std_msgs/Bool<Bool>`
      -  Publish the state of a user sound (True: being played, False: no sound played)
   *  -  ``/niryo_robot_sound/volume_state``
      -  :std_msgs:`std_msgs/Int8<Int8>`
      -  Publish the volume of the robot
   *  -  ``/niryo_robot_sound/get_user_sounds``
      -  :ref:`SoundUser<SoundUser (Message)>`
      -  Publish the sound (and their duration) on the robot

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
   *  -  ``/niryo_robot_sound/play_sound_state``
      -  :ref:`SoundStateCommand<SoundStateCommand (Service)>`
      -  Plays a sound corresponding to a state of the robot
   *  -  ``/niryo_robot_sound/play_sound_user``
      -  :ref:`SoundUserCommand<SoundUserCommand (Service)>`
      -  Plays a sound imported on the robot from the user
   *  -  ``/niryo_robot_sound/stop_sound``
      -  :ref:`StopSound<StopSound (Service)>`
      -  Stops a sound being played
   *  -  ``/niryo_robot_sound/delete_sound_user``
      -  :ref:`DeleteSound<DeleteSound (Service)>`
      -  Deletes a sound imported by the user on the robot
   *  -  ``/niryo_robot_sound/set_volume``
      -  :ref:`SetInt<source/stack/high_level/niryo_robot_msgs:SetInt>`
      -  Sets the volume of the robot
   *  -  ``/niryo_robot_sound/send_sound``
      -  :ref:`SendUserSound<SendUserSound (Service)>`
      -  Imports a sound on the robot
   
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
