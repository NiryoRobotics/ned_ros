Examples: Vision
========================
This document shows how to use Ned's Vision Set.

If you want see more about Ned's Vision functions, you can look at :ref:`API - Vision<source/python_ros_wrapper/ros_wrapper_doc:Vision>`.

Beforehand
-------------------------------
To realize the following examples, you need to have
create a workspace.

As the examples start always the same, there is the code you need to
add at the beginning of all of them: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    niryo_robot = NiryoRosWrapper()

    # - Constants
    workspace_name = "workspace_1"  # Robot's Workspace Name

    # The observation pose
    observation_pose = (0.18, 0., 0.35, 0., 1.57, -0.2)
    # The Place pose
    place_pose = (0., -0.25, 0.1, 0., 1.57, -1.57)
    
    # - Main Program
    # Calibrate robot if robot needs calibration
    niryo_robot.calibrate_auto()
    # Changing tool
    niryo_robot.update_tool()


.. toggle::

    .. image:: ../../images/ros/vision_example.*
       :align: center

Simple Vision pick
-------------------------------
This short example shows how to do your first vision pick: ::

    niryo_robot.move_pose(*observation_pose)
    # Trying to pick target using camera
    ret = niryo_robot.vision_pick(workspace_name,
                                  height_offset=0.0,
                                  shape=ObjectShape.ANY,
                                  color=ObjectColor.ANY)
    obj_found, shape_ret, color_ret = ret
    if obj_found:
        niryo_robot.place_from_pose(*place_pose)

    niryo_robot.set_learning_mode(True)