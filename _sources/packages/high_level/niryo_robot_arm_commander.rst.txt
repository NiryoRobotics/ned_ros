Niryo robot arm commander
#########################

This package is made to interact with the arm through :doc:`../third_parties/moveit` and command the robot motion.

All commands are firstly received on the actionlib server which:
 * Handles concurrent requests.
 * Checks if the command can't be processed due to other factors (ex: learning mode).
 * Validates parameters.
 * Calls required controllers and returns appropriate status and message.

It belongs to the ROS namespace: |namespace_emphasize|.

Package Documentation
*********************

.. rosdoc:: /niryo_robot_arm_commander 
    :action_namespace: robot_action
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_arm_commander

_`The ArmMoveCommand message`
-----------------------------

This message is used when you want to send a move goal to the robot via the action server.
You need to fill the **cmd_type** field with one of the listed constant depending on the type of command you need to send.
Then, you can fill the data field corresponding to your **cmd_type**.

For example, if you command type is **EXECUTE_TRAJ**, you need to fill **dist_smoothing** and **list_poses**.

.. literalinclude:: /../niryo_robot_arm_commander/msg/ArmMoveCommand.msg
    :language: python

.. |namespace_emphasize| replace:: ``/niryo_robot_arm_commander``

