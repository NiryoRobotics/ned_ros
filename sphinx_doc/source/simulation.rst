*******************************************
Use Niryo robot through simulation
*******************************************

The simulation allows to control a virtual Ned directly from
your computer.

.. figure:: ../images/simulation_gazebo_1.png
    :alt: Ned in Gazebo Simulation

    Ned in Gazebo Simulation

In this tutorial, you will learn how to setup a robot simulation on a computer.

.. note::
    You can use :niryo_studio_simulation:`Niryo Studio with the simulation<>`.
    To do so, you just have to connect Niryo Studio to "Localhost".

..todo:: attention au lien ci-dessus qui ne fonctionne pas

Simulation environment installation
=========================================

.. attention::
    The whole ROS Stack is developed and tested on ROS **Melodic** which requires
    **Ubuntu 18.04** to run correctly. The using of another ROS version or OS
    may lead to malfunctions of some packages. Please follow the steps in 
    :ref:`source/installation/ubuntu_18:Ubuntu 18 Installation` to install a working environment.

Simulation usage
=========================================

.. important::
    - Hardware features are simulated as if you were using a real robot.
    - The data returned by the faked drivers is arbitrary and immutable. Among this data, 
      you will have : voltage, temperature, error state (always 0), ping (always true), 
      end effector state (immutable)

| The simulation is a powerful tool allowing to test new programs directly on your computer
 which prevents to transfer new code on the robot.
| It also helps for developing purpose → no need to transfer code, compile and restart the robot
 which is way slower than doing it on a desktop computer.

Without physics - No visualization
--------------------------------------

This mode is mainly for simulation and tests purpose, bringing you in the closest state as possible to
a real robot control. It is available for all currently supported architectures. You can access it by using the command: ::

One simulation:
    `roslaunch niryo_robot_bringup niryo_one_simulation.launch`

Ned simulation:
    `roslaunch niryo_robot_bringup niryo_ned_simulation.launch`

Ned2 simulation:
    `roslaunch niryo_robot_bringup niryo_ned2_simulation.launch`

TODO
.. todo:: il reste un todo ? 

Without physics - Visualization
--------------------------------------

The visualization happens with Rviz which is a powerful tool.

Control with trackbar
^^^^^^^^^^^^^^^^^^^^^^^^^^

This visualization allows an easy first control of the robot, and helps to understand
joints disposal. You can access it by using the command: ::

    roslaunch niryo_robot_description display.launch

Rviz should open with a window containing 6 trackbars. Each of these trackbars allows to control
the corresponding joint.

.. figure:: ../images/visu_rviz_trackbar.jpg
    :alt: Ned on Rviz

    Example of trackbars use.

Control with ROS
^^^^^^^^^^^^^^^^^^^^^^^^^^

| Not only Rviz can display the robot, but it can also be linked with ROS controllers to show the robot's actions
 from ROS commands!

| This method can help you debugging ROS topics, services and also, API scripts.

To run it: ::

    roslaunch niryo_robot_bringup desktop_rviz_simulation.launch

.. figure:: ../images/visu_rviz_ros.jpg
    :alt: Ned on Rviz

    Rviz opening, with the robot ready to be controlled with ROS!

    **TODO add parameters**

.. todo:: un todo

With physics - Simulation
--------------------------------------

For the simulation, Ned uses Gazebo, a well known tool among the ROS community.
It allows:

* Collision.
* World creation → A virtual environment in which the robot can deal with objects.
* Gripper & Camera using.

The Niryo Gripper 1 has been replicated in Gazebo.
The Camera is also implemented.

.. note::
    Gazebo also generates camera distortion, which brings the simulation even closer from the reality!

Launch simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^
A specific world has been created to use Ned in Gazebo with 2 workspaces.

To run it: ::

    roslaunch niryo_robot_bringup desktop_gazebo_simulation.launch

.. figure:: ../images/simulation_gazebo_2.jpg
    :alt: Ned on Gazebo

    Gazebo view, with the robot ready to be controlled with ROS!

.. note::
    You can edit Gazebo world to do your own! It's placed in the folder *worlds* of the package
    niryo_robot_gazebo.

Simulation option
^^^^^^^^^^^^^^^^^^^^^^^^^^

The user can disable 3 things by adding the specific string to the command line:

* the Gazebo graphical interface: `gui:=false`.
* the Camera & the Gripper - Vision & Gripper wise functions won't be usable: `gripper_n_camera:=false`.


.. hint::
    Gazebo can be very slow. If your tests do not need Gripper and Camera, consider using Rviz
    to alleviate your CPU.
