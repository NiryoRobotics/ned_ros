Examples: Basics
==========================

In this file, two short programs are implemented & commented in order to
help you understand the philosophy behind the Python ROS Wrapper.

.. danger::
    If you are using the real robot, make sure the environment around is clear.


Your first move joint
---------------------------

The following example shows a first use case.
It's a simple MoveJ. ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()

    # Moving joint
    niryo_robot.move_joints(0.1, -0.2, 0.0, 1.1, -0.5, 0.2)


Code details - First MoveJ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First of all, we indicate to the shell that we are running a Python Script: ::

    #!/usr/bin/env python

Then, we import the API package to be able to access functions: ::

    from niryo_robot_python_ros_wrapper import *

Then, we install a ROS Node in order to communicate with ROS master: ::

    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

We start a :class:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper` instance: ::

    niryo_robot = NiryoRosWrapper()

Once the connection is done, we calibrate the robot using its
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.calibrate_auto` function: ::

    niryo_robot.calibrate_auto()

As the robot is now calibrated, we can do a Move Joints by giving the 6 axis positions
in radians! To do so, we use :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move_joints`: ::

    niryo_robot.move_joints(0.1, -0.2, 0.0, 1.1, -0.5, 0.2)


Your first pick and place
-------------------------------
For our second example, we are going to develop an algorithm of
pick and place: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_robot_example_python_ros_wrapper')

    # Connecting to the ROS Wrapper & calibrating if needed
    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()

    # Updating tool
    niryo_robot.update_tool()

    # Opening Gripper/Pushing Air
    niryo_robot.release_with_tool()
    # Going to pick pose
    niryo_robot.move_pose(0.2, 0.1, 0.14, 0.0, 1.57, 0)
    # Picking
    niryo_robot.grasp_with_tool()
    # Moving to place pose
    niryo_robot.move_pose(0.2, -0.1, 0.14, 0.0, 1.57, 0)
    # Placing !
    niryo_robot.release_with_tool()

Code details - first pick and place
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First of all, we do the imports and start a ROS Node: ::

    #!/usr/bin/env python

    from niryo_robot_python_ros_wrapper import *
    import rospy

    rospy.init_node('niryo_robot_example_python_ros_wrapper')

Then, create a :class:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper` instance
& calibrate the robot: ::

    niryo_robot = NiryoRosWrapper()
    niryo_robot.calibrate_auto()


Then, we equip the tool
with :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.update_tool` ::

    niryo_robot.update_tool()

Now that our initialization is done, we can open the Gripper (or push air from the Vacuum pump)
with :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.release_with_tool`,
go to the picking pose via :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move_pose`
& then catch an object
with :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.grasp_with_tool`! ::

    # Opening Gripper/Pushing Air
    niryo_robot.release_with_tool()
    # Going to pick pose
    niryo_robot.move_pose(0.2, 0.1, 0.14, 0.0, 1.57, 0)
    # Picking
    niryo_robot.grasp_with_tool()

We now get to the place pose, and place the object. ::

    # Moving to place pose
    niryo_robot.move_pose(0.2, -0.1, 0.14, 0.0, 1.57, 0)
    # Placing !
    niryo_robot.release_with_tool()


Notes - Basics examples
---------------------------
| You may not have fully understood how to move the robot and use
 tools of Ned and that is totally fine because you will find
 more details on another examples page!
| The important thing to remember from this page is how to import the library & connect
 to the robot.
