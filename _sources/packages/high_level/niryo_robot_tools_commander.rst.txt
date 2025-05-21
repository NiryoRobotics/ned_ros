Niryo robot tools commander
###########################

Provides functionalities to setup, control and get feedbacks from the end-effector and accessories for Ned robots.

This package allows to manage the TCP (Tool Center Point) of the robot.
If the functionality is activated, all the movements (in Cartesian coordinates [x, y, z, roll, pitch, yaw]) of the robot will be performed according to this TCP.
The same program can then work with several tools by adapting the TCP transformation to them.
By default this feature is disabled, but can be enabled through a service.

It belongs to the ROS namespace: |namespace_emphasize|.

Tool IDs
********

Each Niryo tool is bound to a specific ID in order to be correctly recognised on the TTL bus.

Here is the list of the tools and their corresponding IDs:

.. list-table:: Niryo Tool IDs Table
   :header-rows: 1
   :widths: auto

   *  -  Name
      -  ID
      -  Image
   *  -  Standard gripper
      -  11
      -  .. image:: /.static/images/tools/standard_gripper.jpg
            :width: 100px
            :alt: Standard gripper
   *  -  Large gripper
      -  12
      -  .. image:: /.static/images/tools/large_gripper.jpg
            :width: 100px
            :alt: Large gripper
   *  -  Adaptive gripper
      -  13
      -  .. image:: /.static/images/tools/adaptive_gripper.jpg
            :width: 100px
            :alt: Adaptive gripper
   *  -  Electromagnet
      -  30
      -  .. image:: /.static/images/tools/electromagnet.jpg
            :width: 100px
            :alt: Electromagnet
   *  -  Vacuum pump
      -  31
      -  .. image:: /.static/images/tools/vacuum_pump.jpg
            :width: 100px
            :alt: Vacuum pump
   *  -  Vacuum pump v2
      -  32
      -  .. image:: /.static/images/tools/vacuum_pump_v2.jpg
            :width: 100px
            :alt: Vacuum pump v2

Tools commander API functions
*****************************

Tools commander ROS wrapper
---------------------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script playing sound via Python ROS Wrapper will looks like: 

.. code:: python

    from niryo_robot_tools_commander.api import ToolsRosWrapper, ToolID

    tools = ToolsRosWrapper()
    tools.update_tool() # Detect and equip the current tool
    tools.open_gripper() # Supposing you have connected a gripper

| This class allows you to control the robot tools via the internal API.

API list
--------

.. automodule:: niryo_robot_tools_commander.api.tools_ros_wrapper
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

.. automodule:: niryo_robot_tools_commander.api.tools_ros_wrapper_enums
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Package Documentation
*********************

.. rosdoc:: /niryo_robot_tools_commander
    :action_namespace: action_server
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_tools_commander


The ToolCommand message
-----------------------

This message is used when you want to send a goal to the robot's end-effector via the action server.
You need to fill the **cmd_type** field with one of the listed constant depending on the type of command you need to send and the correct **tool_id**.
Then, you can fill the data field corresponding to your **cmd_type**.

For example, if you command type is **OPEN_GRIPPER**, you need to fill **max_torque_percentage** and **hold_torque_percentage**.

.. literalinclude:: /../niryo_robot_tools_commander/msg/ToolCommand.msg
    :language: python

.. |namespace_emphasize| replace:: ``/niryo_robot_tools_commander``