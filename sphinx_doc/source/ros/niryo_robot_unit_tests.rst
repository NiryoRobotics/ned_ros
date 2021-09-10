Niryo_robot_unit_tests
========================================
This package provides config and launches files to start Ned unit tests.

Currently, tested packages are:
 - :doc:`niryo_robot_poses_handlers`
 - :doc:`niryo_robot_programs_manager`
 - :doc:`Python ROS Wrapper <../ros_wrapper>`


Run tests with launch file
---------------------------------------------

Using a launch file make the task easy as you do not have to bother about
your current directory. Nevertheless, it starts a roscore, and so, the process 
won't end as "/rosout" will be still alive.
To overcome this issue, the test node is set as **required**, so do not worry
if you see some red lines at the execution's end.

Launch file with Rviz
^^^^^^^^^^^^^^^^^^^^^^^
It will start the ROS Stack and Rviz, then run the tests: ::

 roslaunch niryo_robot_unit_tests simulation_rviz_tests.launch

Launch file with Gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
It will start the ROS Stack and Gazebo, then run the tests.
Gazebo add Physics, Gripper and Camera to the tests: ::

 roslaunch niryo_robot_unit_tests simulation_gazebo_tests.launch

Launch file Headless
^^^^^^^^^^^^^^^^^^^^^^^^
It will start the ROS Stack then run the tests in a headless
version (no display). It is very useful for CI purposes: ::

 roslaunch niryo_robot_unit_tests simulation_headless_tests.launch

Run tests With Python file
---------------------------------------------
Using a python file implies to use the correct relative directory, but
it also allows to get status code at the end of the script.

Python file with Rviz
^^^^^^^^^^^^^^^^^^^^^^^
::

 python niryo_robot_unit_tests/scripts/script_test_rviz.py

Python file with Gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^
::

 python niryo_robot_unit_tests/scripts/script_test_gazebo.py

Python file Headless
^^^^^^^^^^^^^^^^^^^^^^^^
::

 python niryo_robot_unit_tests/scripts/script_test_headless.py

