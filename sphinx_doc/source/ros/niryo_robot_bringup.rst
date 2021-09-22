Niryo_robot_bringup
==============================
This packages provides config and launch files to start Ned and ROS packages with various parameters.

Launch files are placed in the *launch* folder. Only files with **.launch** extension can be executed.


.. figure:: ../../images/ros/brinup_organization.png
   :alt: Bringup organization
   :height: 300px
   :align: center

   Bring Up Launch Files' organization

On RaspberryPI
-------------------------

Ned
^^^^^^^^^^^^^^^^^^

| The file **niryo_ned_robot.launch** allows to launch ROS on a Raspberry Pi 4.
| This file is automatically launched when Ned boots (Ned RPi4B image).

Command to launch Ned's ROS Stack:

 roslaunch niryo_robot_bringup niryo_ned_robot.launch

On Desktop (Simulation)
-------------------------

As the simulation happens on a computer, the hardware-related stuff is not used.

For both of following launch files, you can set:
 - *gui* to "false" in order to disable graphical interface.

Gazebo simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run Gazebo simulation. The robot can do everything that is not hardware-related:
 - move, get_pose.
 - use the camera (to disable it, set "camera" parameter to 'false').
 - use the Gripper 1 (to disable it, set "simu_gripper" parameter to 'false').
 - save/run programs, go to saved pose, ...

Command to launch the simulation: ::

 roslaunch niryo_robot_bringup desktop_gazebo_simulation.launch

To disable camera & gripper: ::

 roslaunch niryo_robot_bringup desktop_gazebo_simulation.launch gripper_n_camera:=false

Rviz simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run Rviz simulation. You can access same features as Gazebo except Camera & Gripper.

To run it, use the command: ::

 roslaunch niryo_robot_bringup desktop_rviz_simulation.launch

Notes - Ned Bringup
------------------------------
*niryo_robot_base* files setup many rosparams,
these files should be launched before any other package.
