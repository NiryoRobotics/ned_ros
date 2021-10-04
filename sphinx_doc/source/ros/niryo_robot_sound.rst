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
      -  Play a sound corresponding to a state of the robot
   *  -  ``/niryo_robot_sound/play_sound_user``
      -  :ref:`SoundUserCommand<SoundUserCommand (Service)>`
      -  Play a sound imported on the robot from the user
   *  -  ``/niryo_robot_sound/stop_sound``
      -  :ref:`StopSound<StopSound (Service)>`
      -  Stop a sound being played
   *  -  ``/niryo_robot_sound/delete_sound_user``
      -  :ref:`DeleteSound<DeleteSound (Service)>`
      -  Delete a sound imported by the user on the robot
   *  -  ``/niryo_robot_sound/set_volume``
      -  :ref:`SetInt<SetInt>`
      -  Set the volume of the robot
   *  -  ``/niryo_robot_sound/send_sound``
      -  :ref:`SendUserSound<SendUserSound (Service)>`
      -  Import a sound on the robot
   
Dependencies - Sound
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`std_msgs`
- :ref:`niryo_robot_msgs <Niryo Robot Messages Package>`
- :ref:`niryo_robot_status <Niryo Robot Status Package>`

Services & Messages files - Sound
----------------------------------------------

SoundName (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/SoundName.msg
   :language: rostype

SoundObject (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/SoundObject.msg
   :language: rostype

SoundUser (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/msg/SoundUser.msg
   :language: rostype
   
DeleteSound (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/DeleteSound.srv
   :language: rostype

SendUserSound (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/SendUserSound.srv
   :language: rostype

SoundStateCommand (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/SoundStateCommand.srv
   :language: rostype

SoundUserCommand (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_msgs/srv/SoundUserCommand.srv
   :language: rostype

StopSound (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   
.. literalinclude:: ../../../niryo_robot_msgs/srv/StopSound.srv
   :language: rostype

.. |namespace| replace:: /niryo_robot_sound/
.. |namespace_emphasize| replace:: ``/niryo_robot_sound/``
.. |package_path| replace:: ../../../niryo_robot_sound