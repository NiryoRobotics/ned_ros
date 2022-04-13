Examples: Dynamic frames
============================

This document shows how to use dynamic frames.

If you want to see more about dynamic frames functions, you can look at :ref:`API - Dynamic frames<source/python_ros_wrapper/ros_wrapper_doc:Dynamic frames>`

.. danger::
    If you are using the real robot, make sure the environment around it is clear.

Simple dynamic frame control
-------------------------------
This example shows how to create a frame and do a small pick and place in this frame: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    gripper_speed = 400

    # Initializing ROS node
    rospy.init_node('niryo_example_python_ros_wrapper')

    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()

    # Create frame
    point_o = [0.15, 0.15, 0]
    point_x = [0.25, 0.2, 0]
    point_y = [0.2, 0.25, 0]

    niryo_robot.save_dynamic_frame_from_points("dynamic_frame", "description", [point_o, point_x, point_y])

    # Get list of frames
    print(niryo_robot.get_saved_dynamic_frame_list())
    # Check creation of the frame
    info = niryo_robot.get_saved_dynamic_frame("dynamic_frame")
    print(info)

    # Pick
    #niryo_robot.open_gripper(gripper_speed)
    # Move to the frame
    niryo_robot.move_pose(0, 0, 0, 0, 1.57, 0, "dynamic_frame")
    #niryo_robot.close_gripper(gripper_speed)

    # Move in frame
    niryo_robot.move_linear_relative([0, 0, 0.1, 0, 0, 0], "dynamic_frame")
    niryo_robot.move_relative([0.1, 0, 0, 0, 0, 0], "dynamic_frame")
    niryo_robot.move_linear_relative([0, 0, -0.1, 0, 0, 0], "dynamic_frame")

    # Place
    #niryo_robot.open_gripper(gripper_speed)
    niryo_robot.move_linear_relative([0, 0, 0.1, 0, 0, 0], "dynamic_frame")

    # Home
    niryo_robot.move_joints(0, 0.5, -1.25, 0, 0, 0)

    # Delete frame
    niryo_robot.delete_dynamic_frame("dynamic_frame")
