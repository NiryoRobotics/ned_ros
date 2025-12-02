Niryo robot programs manager v2
###############################

.. note::
   This package is the new version of the former :doc:`./niryo_robot_programs_manager` package who was in charge of is in charge of interpreting/running/saving programs. 

It is in charge of interpreting/running/saving blocky/python programs.

It belongs to the ROS namespace: |namespace_emphasize|.

Autorun program
***************

The autorun program is a specific program which is bound to the `top button <https://docs.niryo.com/robots/ned3-pro/description-technique/#header-three-8okl>`_ of your Ned robot.


Package Documentation
*********************

.. rosdoc:: /niryo_robot_programs_manager_v2 
    :action_namespace: execute_program
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_programs_manager_v2
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_programs_manager_v2/launch/programs_manager.launch

The ExecuteProgram action
--------------------------

This message is used when you want to execute a program to the robot via the action server.
You need to fill the **program_id** with the ID of your saved program or the **code_string** field with a python code if you want to execute a program which is not saved.


.. literalinclude:: /../niryo_robot_programs_manager_v2/action/ExecuteProgram.action
    :language: python

.. |namespace_emphasize| replace:: ``/niryo_robot_programs_manager``
