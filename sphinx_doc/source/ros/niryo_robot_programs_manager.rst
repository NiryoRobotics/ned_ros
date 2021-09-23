Niryo_robot_programs_manager
======================================================

This package is in charge of interpreting/running/saving programs.
It is used by Niryo Studio.


Programs manager node
--------------------------

The ROS Node is made of several services to deal with the storage and running of
programs.

Calls are not available from the Python ROS Wrapper, as it made to run its programs
with the Python ROS Wrapper.

The namespace used is: |namespace_emphasize|

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
      -  :ref:`ExecuteProgram<source/ros/niryo_robot_programs_manager:ExecuteProgram (Service)>`
      -  Executes a program
   *  -  ``execute_program_autorun``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Executes autorun program
   *  -  ``get_program``
      -  :ref:`GetProgram<source/ros/niryo_robot_programs_manager:GetProgram (Service)>`
      -  Retrieves saved program
   *  -  ``get_program_autorun_infos``
      -  :ref:`GetProgramAutorunInfos<source/ros/niryo_robot_programs_manager:GetProgramAutorunInfos (Service)>`
      -  Gets autorun settings
   *  -  ``get_program_list``
      -  :ref:`GetProgramList<source/ros/niryo_robot_programs_manager:GetProgramList (Service)>`
      -  Gets saved programs' name
   *  -  ``manage_program``
      -  :ref:`ManageProgram<source/ros/niryo_robot_programs_manager:ManageProgram (Service)>`
      -  Saves and Deletes programs
   *  -  ``set_program_autorun``
      -  :ref:`SetProgramAutorun<source/ros/niryo_robot_programs_manager:SetProgramAutorun (Service)>`
      -  Sets autorun settings
   *  -  ``stop_program``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Stops the current running program


All these services are available as soon as the node is started
whereas on standalone mode or not.

Dependencies - Programs manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :doc:`niryo_robot_msgs`
- `python-yaml <https://pyyaml.org/wiki/PyYAMLDocumentation>`_
- :msgs_index:`std_msgs`


Services & messages files - Programs manager
----------------------------------------------

ExecuteProgram (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/srv/ExecuteProgram.srv
   :language: rostype


GetProgram (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/srv/GetProgram.srv
   :language: rostype


GetProgramAutorunInfos (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/srv/GetProgramAutorunInfos.srv
   :language: rostype


GetProgramList (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/srv/GetProgramList.srv
   :language: rostype


ManageProgram (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/srv/ManageProgram.srv
   :language: rostype


SetProgramAutorun (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/srv/SetProgramAutorun.srv
   :language: rostype


ProgramLanguage (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/msg/ProgramLanguage.msg
   :language: rostype


ProgramLanguageList (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/msg/ProgramLanguageList.msg
   :language: rostype


ProgramList (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_programs_manager/msg/ProgramList.msg
   :language: rostype



.. |namespace| replace:: /niryo_robot_programs_manager/
.. |namespace_emphasize| replace:: ``/niryo_robot_programs_manager/``
.. |package_path| replace:: ../../../niryo_robot_programs_manager
