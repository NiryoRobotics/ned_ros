Niryo robot programs manager
############################

This package is in charge of interpreting/running/saving programs.
It is used by the former version of Niryo Studio (<= v4.1.2).


Programs manager node
*********************

The ROS Node is made of several services to deal with the storage and running of
programs.

Calls are not available from the Python ROS Wrapper, as it is made to run its programs
with the Python ROS Wrapper.

It belongs to the ROS namespace: |namespace_emphasize|.

Package Documentation
*********************

.. rosdoc:: /niryo_robot_programs_manager
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_programs_manager
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_programs_manager/launch/programs_manager.launch

.. |namespace_emphasize| replace:: ``/niryo_robot_programs_manager``