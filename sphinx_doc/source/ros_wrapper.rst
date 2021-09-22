Control with Python ROS Wrapper
====================================================

.. figure:: ../images/python_logo.png
   :alt: Python Logo
   :width: 400px
   :align: center

   Python Logo

In order to control Ned more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script realizing a moveJ via Python ROS Wrapper will look like: ::

    niryo_robot = NiryoRosWrapper()
    niryo_robot.move_joints(0.1, -0.2, 0.0, 1.1, -0.5, 0.2)


What this code is doing in a hidden way:

 - It generates a :ref:`RobotMove Action Goal<source/ros/niryo_robot_arm_commander:RobotMove (Action)>` and set
   it as a joint command with the corresponding joints value.
 - Sends goal to the Commander Action Server.
 - Waits for the Commander Action Server to set Action as finished.
 - Checks if action finished with a success.


In this section, we will give some examples on how to use the Python ROS Wrapper to control
Ned, as well as a complete documentation of the functions available in the
Ned Python ROS Wrapper.

.. hint::
    The Python ROS Wrapper forces the user to write his code directly in the robot, or, at least,
    copy the code on the robot via a terminal command.
    If you do not want that, and run code directly from your computer
    you can use the python Package :ref:`source/more:PyNiryo`.


.. toctree::
   :maxdepth: 2
   :caption: Section Contents:

   python_ros_wrapper/before_running
   python_ros_wrapper/examples_basics
   python_ros_wrapper/examples_movement
   python_ros_wrapper/examples_tool_action
   python_ros_wrapper/examples_conveyor
   python_ros_wrapper/examples_vision
   python_ros_wrapper/ros_wrapper_doc
