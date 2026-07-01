Examples: Motions
#################

This document shows how to control Ned with Move Joints & Move Pose.

If you want see more about Ned robot's arm commander package, go to :doc:`Niryo robot arm commander </packages/high_level/niryo_robot_arm_commander>`

.. danger::
    If you are using the real robot, make sure the environment around is clear.

Joints
******

To do a move joints, you have to call the :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move` method with a
:class:`~.niryo_robot_utils.dataclasses.JointsPosition.JointsPosition` object. A ``JointsPosition`` object contains a list of the 6 joints goal positions.

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:

To get the current joint positions, we use :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.get_joints`:

.. code:: python

    joints = robot.get_joints()
    j1, j2, j3, j4, j5, j6 = joints

Poses
*****

To do a move pose, you have to call the :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move` method with a
:class:`~.niryo_robot_utils.dataclasses.Pose.Pose` object. A ``Pose`` object contains a list of (x, y, z) values for the end-effector position and (yaw, pitch, roll) for its orientation.


.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_pose.py
  :language: python
  :linenos:


To get the current pose, we use :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.get_pose`:

.. code:: python

     robot_state = niryo_robot.get_pose()


