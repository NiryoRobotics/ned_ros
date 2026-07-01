Examples: Tool action
#####################

This document shows how to control Ned robot's tools via the Python ROS Wrapper.

If you want see more about Ned robot's tool package, go to :doc:`Niryo robot tools commander </packages/high_level/niryo_robot_tools_commander>`

.. danger::
    If you are using the real robot, make sure the environment around it is clear.

Tool control
************

Setup tool
^^^^^^^^^^

In order to use a tool, it should be mechanically plugged to the robot but also recognized by the robot software.

To do that, we must use the function
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.update_tool`. It will scan motor connections and set the new tool.

.. code:: python

    robot.update_tool()

Grasp
^^^^^

To grasp with any tool, you can use the function:
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.grasp_with_tool`. This action can, depending on the tool, correspond to:

 - close the gripper
 - pull Air from a vacuum pump
 - activate an electromagnet

Simple grasping example:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/tool_grasp.py
  :language: python
  :linenos:

Advanced grasping with parameters example:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/tool_grasp_with_parameters.py
  :language: python
  :linenos:

Release
^^^^^^^

To release with any tool, you can use the function:
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.release_with_tool`. This action can, depending on the tool, correspond to:

 - open the gripper
 - push Air from a vacuum pump
 - deactivate an electromagnet

Simple releasing example:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/tool_release.py
  :language: python
  :linenos:

Advanced releasing with parameters example:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/tool_release_with_parameters.py
  :language: python
  :linenos:
