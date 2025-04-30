Control with Python ROS Wrapper
###############################

.. figure:: /.static/images/python_logo.png
   :alt: Python Logo
   :width: 400px
   :align: center

In order to control Ned robot more easily than calling ROS topics & services,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script realizing a moveJ via Python ROS Wrapper will looks like:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:

What the underlying code does:

 - It generates a RobotMove Action Goal (cf :doc:`this package <high_level/niryo_robot_arm_commander>` for more informations) and set
   it as a joint command with the corresponding joints value.
 - Sends the goal to the Commander Action Server.
 - Waits for the Commander Action Server to set Action as finished.
 - Checks if the action finished successfully.


In this section, we will give some examples on how to use the Python ROS Wrapper to control
Ned robots, as well as a complete documentation of the functions available in the
Ned Python ROS Wrapper.

.. hint::
    The Python ROS Wrapper forces the user to write his code directly in the robot, or, at least,
    copy the code on the robot via a terminal command.
    If you do not want to do that, and run code directly from your computer
    you can use the :pyniyro:`Pyniryo <>` python package.


.. toctree::
   :maxdepth: 2
   :caption: Section Contents:

   python_ros_wrapper/ros_wrapper_api
   python_ros_wrapper/before_running
   python_ros_wrapper/examples_basics
   python_ros_wrapper/examples_motions
   python_ros_wrapper/examples_tool_action
   python_ros_wrapper/examples_conveyor
   python_ros_wrapper/examples_vision
   python_ros_wrapper/examples_frames
   
