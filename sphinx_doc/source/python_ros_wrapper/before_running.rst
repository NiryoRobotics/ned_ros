Before running your programs
====================================================

The variable PYTHONPATH
---------------------------------
The Python interpreter needs to have all used packages in the environment variable **PYTHONPATH**,
to do that, you need to have sourced your ROS environment:

- If you are coding directly on your robot, it is made directly in every terminal.
- If your are using simulation, be sure to have followed the setup from
  :ref:`source/simulation:Setup Ned ROS Environment`.


Required piece of code
-------------------------------

To run, your program will need some imports & initialization. We give you below the piece
of code you must use to make Python ROS Wrapper work: ::

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import *
    import rospy

    # Initializing ROS node
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    niryo_robot = NiryoRosWrapper()

    # -- YOUR CODE HERE -- #


You have now everything you need to control the robot through its Python ROS Wrapper. To run
a script, simply use the command ``python my_script.py``.

