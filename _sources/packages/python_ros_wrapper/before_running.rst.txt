Before running your programs
############################

The variable PYTHONPATH
***********************
The Python interpreter needs to have all used packages in the environment variable **PYTHONPATH**,
to do that, you need to source your ROS workspace:

- If you are coding directly on your robot, it is already done.
- If you are using the simulation on your computer, make sure to follow the setup environment setup from
  :doc:`Ubuntu 20.04 Installation </installation/install_for_ubuntu_20>`.


Required piece of code
**********************

To run, your program will need some imports & initialization. We give you below the piece
of code you must use to make Python ROS Wrapper works:

.. code:: python

    #!/usr/bin/env python

    # Imports
    from niryo_robot_python_ros_wrapper import NiryoRosWrapper

    # Instantiate the ROS wrapper and initialize the ROS node
    robot = NiryoRosWrapper.init_with_node()

    # -- YOUR CODE HERE -- #


You have now everything you need to control the robot through its Python ROS Wrapper. To run
a script, simply use the command ``python3 my_script.py``.

