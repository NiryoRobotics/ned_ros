Examples: tool action
========================

This page shows how to control Ned's tools via the Python ROS Wrapper.

If you want see more, you can look at :ref:`API - Tools<source/python_ros_wrapper/ros_wrapper_doc:Tools>`.

.. danger::
    If you are using the real robot, make sure the environment around it is clear.

Tool control
-------------------

Equip tool
^^^^^^^^^^^^

In order to use a tool, it should be mechanically plugged to the robot but also
connected software wise.

To do that, we should use the function
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.update_tool`
which takes no argument. It will scan motor connections and set the new tool!

The line to equip a new tool is: ::

    niryo_robot.update_tool()



Grasping
^^^^^^^^^^^^

To grasp with any tool, you can use the function:
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.grasp_with_tool`. This action corresponds to:

 - Close gripper for Grippers.
 - Pull Air for Vacuum pump.
 - Activate for Electromagnet.

The code to grasp is: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()

    # Updating tool
    niryo_robot.update_tool()

    # Grasping
    niryo_robot.grasp_with_tool()


To grasp by specifying parameters: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()

    # Updating tool
    tool_used = ToolID.XXX
    niryo_robot.update_tool()

    if tool_used in [ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4]:
        niryo_robot.close_gripper(speed=500)
    elif tool_used == ToolID.ELECTROMAGNET_1:
        pin_electromagnet = PinID.XXX
        niryo_robot.setup_electromagnet(pin_electromagnet)
        niryo_robot.activate_electromagnet(pin_electromagnet)
    elif tool_used == ToolID.VACUUM_PUMP_1:
        niryo_robot.pull_air_vacuum_pump()


Releasing
^^^^^^^^^^^^

To release with any tool, you can use the function:
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.release_with_tool`. This action correspond to:

  - Open gripper for Grippers.
  - Push Air for Vacuum pump.
  - Deactivate for Electromagnet.

The line to release is: ::

    niryo_robot.release_with_tool()

To release by specifying parameters: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()

    # Updating tool
    tool_used = ToolID.XXX
    niryo_robot.update_tool()

    if tool_used in [ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4]:
        niryo_robot.open_gripper(speed=500)
    elif tool_used == ToolID.ELECTROMAGNET_1:
        pin_electromagnet = PinID.XXX
        niryo_robot.setup_electromagnet(pin_electromagnet)
        niryo_robot.deactivate_electromagnet(pin_electromagnet)
    elif tool_used == ToolID.VACUUM_PUMP_1:
        niryo_robot.push_air_vacuum_pump(tool_used)



Pick & place with tools
-------------------------

There are a plenty of ways to realize a pick and place with the ROS Wrapper. Methods will
be presented from the lowest to highest level.

Code used will be: ::

    # Imports
    from niryo_robot_python_ros_wrapper import *
    
    gripper_used = ToolID.XXX  # Tool used for picking

    # The pick pose
    pick_pose = (0.25, 0., 0.15, 0., 1.57, 0.0)
    # The Place pose
    place_pose = (0., -0.25, 0.1, 0., 1.57, -1.57)
    
    def pick_n_place_version_x(niryo_ned):
        # -- SOME CODE -- #
    
    if __name__ == '__main__':
        niryo_robot = NiryoRosWrapper()
        niryo_robot.calibrate_auto()
        pick_n_place_version_x(niryo_robot)
    

First solution: the heaviest
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Everything is done by hand: ::

    def pick_n_place_version_1(niryo_ned):
        height_offset = 0.05  # Offset according to Z-Axis to go over pick & place poses
        gripper_speed = 400
    
        # Going Over Object
        niryo_ned.move_pose(pick_pose[0], pick_pose[1], pick_pose[2] + height_offset,
                            pick_pose[3], pick_pose[4], pick_pose[5])
        # Opening Gripper
        niryo_ned.open_gripper(gripper_speed)
        # Going to picking place and closing gripper
        niryo_ned.move_pose(pick_pose[0], pick_pose[1], pick_pose[2],
                            pick_pose[3], pick_pose[4], pick_pose[5])
        niryo_ned.close_gripper(gripper_speed)
    
        # Raising
        niryo_ned.move_pose(pick_pose[0], pick_pose[1], pick_pose[2] + height_offset,
                            pick_pose[3], pick_pose[4], pick_pose[5])
    
        # Going Over Place pose
        niryo_ned.move_pose(place_pose[0], place_pose[1], place_pose[2] + height_offset,
                            place_pose[3], place_pose[4], place_pose[5])
        # Going to Place pose
        niryo_ned.move_pose(place_pose[0], place_pose[1], place_pose[2],
                            place_pose[3], place_pose[4], place_pose[5])
        # Opening Gripper
        niryo_ned.open_gripper(gripper_speed)
        # Raising
        niryo_ned.move_pose(place_pose[0], place_pose[1], place_pose[2] + height_offset,
                            place_pose[3], place_pose[4], place_pose[5])


Second solution: pick from pose & place from pose functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
We use predefined functions: ::

    def pick_n_place_version_3(niryo_ned):
        # Pick
        niryo_ned.pick_from_pose(*pick_pose)
        # Place
        niryo_ned.place_from_pose(*place_pose)

Third solution: all in one
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
We use THE predifined function: ::

    def pick_n_place_version_4(niryo_ned):
        # Pick & Place
        niryo_ned.pick_and_place(pick_pose, place_pose)

