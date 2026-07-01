Niryo robot gazebo
##################


.. figure:: ../../.static/images/gazebo_logo.png
   :alt: Gazebo logo
   :height: 200px
   :align: center

Usage
*****
This package contains models, materials & Gazebo worlds.

When launching the Gazebo version of the ROS Stack, the file
**niryo_robot_gazebo_world.launch.xml** will be called to generate the Gazebo world.


Create your own world
*********************

Create your world's file and put it on the folder *worlds*. Once it is done,
you have to change the parameter **world_name** in the file
**niryo_robot_gazebo_world.launch.xml**.

You can take a look at the Gazebo world by launching it without robot by precising
the world name in the arg *world_name*:

.. code:: bash
    
    roslaunch niryo_robot_gazebo niryo_gazebo_world.launch world_name:=niryo_cube_world hardware_version:=ned2  # ned2, ned3pro


