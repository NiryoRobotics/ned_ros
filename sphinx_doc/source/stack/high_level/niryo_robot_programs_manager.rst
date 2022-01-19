Niryo_robot_programs_manager
======================================================

This package is in charge of interpreting/running/saving programs.
It is used by Niryo Studio.


Programs manager node
--------------------------

The ROS Node is made of several services to deal with the storage and running of
programs.

Calls are not available from the Python ROS Wrapper, as it is made to run its programs
with the Python ROS Wrapper.

It belongs to the ROS namespace: |namespace_emphasize|.

Parameters - Programs manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: Programs Manager's Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``autorun_file_name``
      -  Name of the file containing auto run infos
   *  -  ``programs_dir``
      -  Path to the Program storage mother folder

Services - Programs manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Programs manager Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``execute_program``
      -  :ref:`ExecuteProgram<source/stack/high_level/niryo_robot_programs_manager:ExecuteProgram>`
      -  Executes a program
   *  -  ``execute_program_autorun``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Executes autorun program
   *  -  ``get_program``
      -  :ref:`GetProgram<source/stack/high_level/niryo_robot_programs_manager:GetProgram>`
      -  Retrieves saved program
   *  -  ``get_program_autorun_infos``
      -  :ref:`GetProgramAutorunInfos<source/stack/high_level/niryo_robot_programs_manager:GetProgramAutorunInfos>`
      -  Gets autorun settings
   *  -  ``get_program_list``
      -  :ref:`GetProgramList<source/stack/high_level/niryo_robot_programs_manager:GetProgramList>`
      -  Gets saved programs' name
   *  -  ``manage_program``
      -  :ref:`ManageProgram<source/stack/high_level/niryo_robot_programs_manager:ManageProgram>`
      -  Saves and Deletes programs
   *  -  ``set_program_autorun``
      -  :ref:`SetProgramAutorun<source/stack/high_level/niryo_robot_programs_manager:SetProgramAutorun>`
      -  Sets autorun settings
   *  -  ``stop_program``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Stops the current running program


All these services are available as soon as the node is started
whereas on standalone mode or not.

Dependencies - Programs manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :doc:`niryo_robot_msgs`
- `python-yaml <https://pyyaml.org/wiki/PyYAMLDocumentation>`_
- :msgs_index:`std_msgs`


Services files - Programs manager
----------------------------------------------

ExecuteProgram
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/srv/ExecuteProgram.srv
   :language: rostype


GetProgram
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/srv/GetProgram.srv
   :language: rostype


GetProgramAutorunInfos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/srv/GetProgramAutorunInfos.srv
   :language: rostype


GetProgramList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/srv/GetProgramList.srv
   :language: rostype


ManageProgram
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/srv/ManageProgram.srv
   :language: rostype


SetProgramAutorun
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/srv/SetProgramAutorun.srv
   :language: rostype


Messages files - Programs manager
----------------------------------------------

ProgramIsRunning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/msg/ProgramIsRunning.msg
   :language: rostype


ProgramLanguage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/msg/ProgramLanguage.msg
   :language: rostype


ProgramLanguageList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/msg/ProgramLanguageList.msg
   :language: rostype


ProgramList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_programs_manager/msg/ProgramList.msg
   :language: rostype



.. |namespace| replace:: /niryo_robot_programs_manager/
.. |namespace_emphasize| replace:: ``/niryo_robot_programs_manager/``
.. |package_path| replace:: ../../../../niryo_robot_programs_manager
