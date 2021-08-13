Examples: Movement
=========================

This document shows how to control Ned in order to make Move Joints & Move Pose.

If you want see more, you can look at :ref:`API - Joints & Pose<source/python_ros_wrapper/ros_wrapper_doc:Joints & Pose>`.

.. danger::
    If you are using the real robot, make sure the environment around is clear.

Joints
-------------------

To do a moveJ, you should pass 6 floats: (j1, j2, j3, j4, j5, j6) to the
method :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move_joints`: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    niryo_robot = NiryoRosWrapper()
    niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

To get joints, we use :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.get_joints`: ::

    joints = niryo_robot.get_joints()
    j1, j2, j3, j4, j5, j6 = joints

Pose
-------------------

To do a moveP, you should pass 6 floats: (x, y, z, roll, pitch, yaw) to the
method :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move_pose`.

See on this example: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    niryo_robot = NiryoRosWrapper()
    niryo_robot.move_pose(0.25, 0.0, 0.25, 0.0, 0.0, 0.0)

To get pose, we use :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.get_pose`: ::

    x, y, z, roll, pitch, yaw = niryo_robot.get_pose()


